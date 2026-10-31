// ChestPhysicsAML - mod AML untuk GTA SA Android v2.0 (armeabi-v7a, 32-bit)
// Ditulis semula (clean-room) berdasarkan analisis ChestPhysicsAllPeds.asi.
// STATUS: BELUM DIUJI pada peranti. Baca BACA_SAYA.md dahulu.
#include <mod/amlmod.h>
#include <mod/logger.h>
#include <unordered_map>
#include <vector>
#include <string>
#include <fstream>
#include <cmath>
#include <cstdint>
#include <cstdlib>

MYMODCFG(net.user.chestphysics, ChestPhysicsAML, 0.1.0, ChestPhysics)

struct V3 { float x, y, z; };
struct RwMat { V3 right; uint32_t f0; V3 up; uint32_t f1; V3 at; uint32_t f2; V3 pos; uint32_t f3; };
static_assert(sizeof(RwMat) == 64, "RwMatrix mesti 64 bait");

// eBoneTag (plugin-sdk)
enum { BONE_SPINE1 = 3, BONE_UPPERTORSO = 4, BONE_R_CLAVICLE = 21, BONE_L_CLAVICLE = 31,
       BONE_R_BREAST = 301, BONE_L_BREAST = 302 };

// ---------- Konfigurasi (ChestPhysics.ini) ----------
struct Cfg {
    bool  enabled = true, femaleOnly = true, flex = false;
    float spring = 120.f, damping = 7.f;
    float respV = 0.9f, respH = 0.5f;             // sensitiviti terhadap pecutan (menegak / mendatar)
    float softLimit = 0.03f, hardLimit = 0.05f;   // had anjakan (meter)
    float maxSpeed = 1.2f;                        // had laju anjakan (m/s)
    float softUp = 0.6f, softDown = 1.0f;         // pengganda had untuk arah atas / bawah
    uint32_t pedTypeOffset = 0x598;               // OFFSET CPed::m_nPedType - SAHKAN utk v2.0!
    std::vector<int> femaleTypes = {5, 22};       // CIVFEMALE, PROSTITUTE
} C;

static void LoadIni() {
    const char* dir = aml->GetConfigPath();
    std::string path = std::string(dir ? dir : ".") + "/ChestPhysics.ini";
    std::ifstream f(path);
    if (!f) { logger->Info("ChestPhysics.ini tiada, guna nilai asal (%s)", path.c_str()); return; }
    std::string line;
    while (std::getline(f, line)) {
        if (line.empty() || line[0] == ';' || line[0] == '#' || line[0] == '[') continue;
        size_t eq = line.find('=');
        if (eq == std::string::npos) continue;
        std::string k = line.substr(0, eq), v = line.substr(eq + 1);
        auto trim = [](std::string& s) { size_t a = s.find_first_not_of(" \t\r"), b = s.find_last_not_of(" \t\r");
                                         s = (a == std::string::npos) ? "" : s.substr(a, b - a + 1); };
        trim(k); trim(v);
        float fv = (float)atof(v.c_str());
        if      (k == "enabled")         C.enabled = atoi(v.c_str()) != 0;
        else if (k == "mode")            C.flex = (v == "flex");
        else if (k == "female_only")     C.femaleOnly = atoi(v.c_str()) != 0;
        else if (k == "ped_type_offset") C.pedTypeOffset = (uint32_t)strtoul(v.c_str(), nullptr, 0);
        else if (k == "spring")          C.spring = fv;
        else if (k == "damping")         C.damping = fv;
        else if (k == "response_v")      C.respV = fv;
        else if (k == "response_h")      C.respH = fv;
        else if (k == "soft_limit")      C.softLimit = fv;
        else if (k == "hard_limit")      C.hardLimit = fv;
        else if (k == "max_speed")       C.maxSpeed = fv;
        else if (k == "soft_up")         C.softUp = fv;
        else if (k == "soft_down")       C.softDown = fv;
    }
    if (C.flex) { C.spring *= 0.6f; C.damping *= 0.6f; C.softLimit *= 1.5f; C.hardLimit *= 1.5f; }
}

// ---------- Fungsi permainan (dicari mengikut nama melalui dlsym) ----------
typedef void*  (*GetHier_t)(void* clump);
typedef RwMat* (*GetMatArr_t)(void* hier);
typedef int    (*IdIndex_t)(void* hier, int id);
static GetHier_t   GetHier;
static GetMatArr_t GetMatArr;
static IdIndex_t   IdIndex;
static float*      g_timeStep;     // CTimer::ms_fTimeStep (1.0 = 1/50 saat)
static uint32_t*   g_frame;        // CTimer::m_FrameCounter
static void (*Render_orig)(void*);

// ---------- Matematik ----------
static inline V3 sub(V3 a, V3 b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
static inline V3 mul(V3 a, float s) { return {a.x * s, a.y * s, a.z * s}; }
static inline float dot(V3 a, V3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
static inline V3 cross(V3 a, V3 b) { return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x}; }
static inline V3 norm(V3 a) { float l = sqrtf(dot(a, a)); return l > 1e-6f ? mul(a, 1.f / l) : V3{0, 1, 0}; }

// ---------- Fizik spring-damper ----------
struct Spring { V3 o{0, 0, 0}, u{0, 0, 0}; };
struct PedState { uint32_t frame = 0; bool init = false; V3 prevPos{}, prevVel{}; Spring L, R; };
static std::unordered_map<void*, PedState> g_states;

static void StepSpring(Spring& s, V3 aLocal, float dt, float kMul) {
    const int N = 2; const float h = dt / N, k = C.spring * kMul, c = C.damping;
    const float drive[3] = {aLocal.x * C.respH, aLocal.y * C.respV, aLocal.z * C.respH};
    for (int i = 0; i < N; i++) {
        float* o = &s.o.x; float* u = &s.u.x;
        for (int ax = 0; ax < 3; ax++) {
            float m = (ax == 1) ? (o[ax] >= 0 ? C.softUp : C.softDown) : 1.f;
            float soft = C.softLimit * m, hard = C.hardLimit * m;
            float f = -k * o[ax] - c * u[ax] - drive[ax];            // inersia: badan memecut -> dada ketinggalan
            float ao = fabsf(o[ax]);
            if (ao > soft) f -= (o[ax] > 0 ? 1.f : -1.f) * k * 6.f * (ao - soft);   // had lembut
            u[ax] += f * h;
            if (u[ax] >  C.maxSpeed) u[ax] =  C.maxSpeed;
            if (u[ax] < -C.maxSpeed) u[ax] = -C.maxSpeed;
            o[ax] += u[ax] * h;
            if (fabsf(o[ax]) > hard) { o[ax] = copysignf(hard, o[ax]); if (u[ax] * o[ax] > 0) u[ax] = 0; }  // had keras
        }
    }
}

static bool IsFemale(void* ped) {
    if (!C.femaleOnly) return true;
    int t = *(int*)((char*)ped + C.pedTypeOffset);
    for (int ft : C.femaleTypes) if (t == ft) return true;
    return false;
}

static void Apply(void* ped) {
    if (!IsFemale(ped)) return;
    void* clump = *(void**)((char*)ped + 0x18);          // CEntity::m_pRwObject - SAHKAN utk v2.0
    if (!clump) return;
    PedState& st = g_states[ped];
    uint32_t frame = *g_frame;
    if (st.frame == frame) return;                          // sekali sahaja setiap frame
    void* hier = GetHier(clump); if (!hier) return;
    RwMat* arr = GetMatArr(hier); if (!arr) return;
    int iT = IdIndex(hier, BONE_UPPERTORSO), iS = IdIndex(hier, BONE_SPINE1);
    int iRC = IdIndex(hier, BONE_R_CLAVICLE), iLC = IdIndex(hier, BONE_L_CLAVICLE);
    int iR = IdIndex(hier, BONE_R_BREAST), iL = IdIndex(hier, BONE_L_BREAST);
    if (iT < 0 || iS < 0 || iRC < 0 || iLC < 0 || (iR < 0 && iL < 0)) return;

    // Bingkai tempatan dada dalam ruang dunia (matriks hierarki ialah ruang dunia)
    V3 pT = arr[iT].pos;
    V3 up = norm(sub(pT, arr[iS].pos));
    V3 rt = sub(arr[iRC].pos, arr[iLC].pos); rt = norm(sub(rt, mul(up, dot(rt, up))));
    V3 at = cross(rt, up);

    float dt = (*g_timeStep) / 50.f; if (dt < 0.001f) dt = 0.001f; if (dt > 0.05f) dt = 0.05f;
    if (!st.init || frame - st.frame > 5) { st.prevPos = pT; st.prevVel = {0, 0, 0}; st.L = st.R = Spring(); st.init = true; }
    V3 vel = mul(sub(pT, st.prevPos), 1.f / dt);
    V3 acc = mul(sub(vel, st.prevVel), 1.f / dt);
    float al = sqrtf(dot(acc, acc)); if (al > 300.f) acc = mul(acc, 300.f / al);   // tapis hingar / teleport
    st.prevPos = pT; st.prevVel = vel; st.frame = frame;

    V3 aLocal = {dot(acc, rt), dot(acc, up), dot(acc, at)};
    StepSpring(st.R, aLocal, dt, 1.00f);
    StepSpring(st.L, aLocal, dt, 1.04f);                    // beza kecil supaya tidak bergerak 100% serupa

    auto shift = [&](int idx, const Spring& s) {
        if (idx < 0) return;
        V3 d = {rt.x * s.o.x + up.x * s.o.y + at.x * s.o.z,
                rt.y * s.o.x + up.y * s.o.y + at.y * s.o.z,
                rt.z * s.o.x + up.z * s.o.y + at.z * s.o.z};
        arr[idx].pos = {arr[idx].pos.x + d.x, arr[idx].pos.y + d.y, arr[idx].pos.z + d.z};
    };
    shift(iR, st.R); shift(iL, st.L);

    static uint32_t lastClean = 0;                           // buang ped lama
    if (frame - lastClean > 600) {
        lastClean = frame;
        for (auto it = g_states.begin(); it != g_states.end();)
            it = (frame - it->second.frame > 600) ? g_states.erase(it) : ++it;
    }
}

static void Render_hook(void* ped) {
    if (C.enabled) Apply(ped);
    Render_orig(ped);
}

extern "C" void OnModLoad() {
    logger->SetTag("ChestPhysicsAML");
    LoadIni();
    void* h = aml->GetLibHandle("libGTASA.so");
    if (!h) { logger->Error("libGTASA.so tidak ditemui"); return; }
    GetHier   = (GetHier_t)  aml->GetSym(h, "_Z29GetAnimHierarchyFromSkinClumpP7RpClump");
    GetMatArr = (GetMatArr_t)aml->GetSym(h, "_Z30RpHAnimHierarchyGetMatrixArrayP16RpHAnimHierarchy");
    IdIndex   = (IdIndex_t)  aml->GetSym(h, "_Z17RpHAnimIDGetIndexP16RpHAnimHierarchyi");
    g_timeStep = (float*)    aml->GetSym(h, "_ZN6CTimer12ms_fTimeStepE");
    g_frame    = (uint32_t*) aml->GetSym(h, "_ZN6CTimer14m_FrameCounterE");
    void* render = aml->GetSym(h, "_ZN4CPed6RenderEv");
    if (!GetHier || !GetMatArr || !IdIndex || !g_timeStep || !g_frame || !render) {
        logger->Error("Satu atau lebih simbol tidak ditemui - mod tidak diaktifkan"); return;
    }
    aml->Hook(render, (void*)Render_hook, (void**)&Render_orig);
    logger->Info("Dimuatkan (mode: %s)", C.flex ? "flex" : "classic");
}
