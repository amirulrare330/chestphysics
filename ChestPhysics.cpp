// ChestPhysicsAML - mod AML untuk GTA SA Android v2.0 (armeabi-v7a, 32-bit)
// Ditulis semula (clean-room) berdasarkan analisis ChestPhysicsAllPeds.asi.
// STATUS: BELUM DIUJI pada peranti. Baca BACA_SAYA.md dahulu.
#include <mod/amlmod.h>
#include <mod/logger.h>
#if __has_include(<mod/config.h>)
#include <mod/config.h>
#endif
#include <unordered_map>
#include <vector>
#include <string>
#include <fstream>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstdio>
#include <cstdarg>
#include <sys/mman.h>
#include <unistd.h>

MYMODCFG(net.user.chestphysics, ChestPhysicsAML, 0.1.0, ChestPhysics)

struct V3 { float x, y, z; };
struct RwMat { V3 right; uint32_t f0; V3 up; uint32_t f1; V3 at; uint32_t f2; V3 pos; uint32_t f3; };
static_assert(sizeof(RwMat) == 64, "RwMatrix mesti 64 bait");

// eBoneTag (plugin-sdk)
enum { BONE_SPINE1 = 3, BONE_UPPERTORSO = 4, BONE_R_CLAVICLE = 21, BONE_L_CLAVICLE = 31,
       BONE_R_BREAST = 301, BONE_L_BREAST = 302 };

// ---------- Konfigurasi (ChestPhysics.ini) ----------
struct Cfg {
    int   stage = 2;                              // 0=muat sahaja, 1=hook tanpa fizik, 2=penuh
    bool  enabled = true, femaleOnly = true, flex = false;
    float spring = 120.f, damping = 7.f;
    float respV = 0.9f, respH = 0.5f;             // sensitiviti terhadap pecutan (menegak / mendatar)
    float softLimit = 0.03f, hardLimit = 0.05f;   // had anjakan (meter)
    float maxSpeed = 1.2f;                        // had laju anjakan (m/s)
    float softUp = 0.6f, softDown = 1.0f;         // pengganda had untuk arah atas / bawah
    uint32_t pedTypeOffset = 0x598;               // OFFSET CPed::m_nPedType - SAHKAN utk v2.0!
    std::vector<int> femaleTypes = {5, 22};       // CIVFEMALE, PROSTITUTE
} C;

// ---------- Catatan ke fail (untuk cari punca crash) ----------
static std::string g_logPath;
static bool g_verbose = false;
static void Log(const char* fmt, ...) {
    if (g_logPath.empty()) return;
    FILE* f = fopen(g_logPath.c_str(), "a");
    if (!f) return;
    va_list a; va_start(a, fmt); vfprintf(f, fmt, a); va_end(a);
    fputc('\n', f); fclose(f);
}

static void LoadIni() {
    const char* dir = aml->GetConfigPath();
    std::string path = std::string(dir ? dir : ".") + "/ChestPhysics.ini";
    std::ifstream f(path);
    if (!f) { Log("ini tiada, guna nilai asal (%s)", path.c_str()); return; }
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
        else if (k == "stage")           C.stage = atoi(v.c_str());
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
    if (g_verbose) Log("A: baca jenis ped di offset 0x%x", (unsigned)C.pedTypeOffset);
    int t = *(int*)((char*)ped + C.pedTypeOffset);
    if (g_verbose) Log("A: jenis ped = %d", t);
    for (int ft : C.femaleTypes) if (t == ft) return true;
    return false;
}

static void Apply(void* ped) {
    if (!IsFemale(ped)) return;
    if (g_verbose) Log("B: baca clump di offset 0x18");
    void* clump = *(void**)((char*)ped + 0x18);          // CEntity::m_pRwObject - SAHKAN utk v2.0
    if (g_verbose) Log("B: clump = %p", clump);
    if (!clump || (uintptr_t)clump < 0x10000) return;
    PedState& st = g_states[ped];
    uint32_t frame = *g_frame;
    if (st.frame == frame) return;                          // sekali sahaja setiap frame
    void* hier = GetHier(clump);
    if (g_verbose) Log("C: hierarki = %p", hier);
    if (!hier) return;
    RwMat* arr = GetMatArr(hier);
    if (g_verbose) Log("D: tatasusunan matriks = %p", (void*)arr);
    if (!arr) return;
    int iT = IdIndex(hier, BONE_UPPERTORSO), iS = IdIndex(hier, BONE_SPINE1);
    int iRC = IdIndex(hier, BONE_R_CLAVICLE), iLC = IdIndex(hier, BONE_L_CLAVICLE);
    int iR = IdIndex(hier, BONE_R_BREAST), iL = IdIndex(hier, BONE_L_BREAST);
    if (g_verbose) Log("E: indeks tulang T=%d S=%d RC=%d LC=%d R=%d L=%d", iT, iS, iRC, iLC, iR, iL);
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
    if (g_verbose) Log("F: fizik digunakan pada satu ped");

    static uint32_t lastClean = 0;                           // buang ped lama
    if (frame - lastClean > 600) {
        lastClean = frame;
        for (auto it = g_states.begin(); it != g_states.end();)
            it = (frame - it->second.frame > 600) ? g_states.erase(it) : ++it;
    }
}

static int g_calls = 0;
static void Render_pass(void* ped) {            // peringkat 1: hook sahaja
    if (g_calls < 3) { g_calls++; Log("Render dipanggil (pass) #%d ped=%p", g_calls, ped); }
    Render_orig(ped);
}
static void Render_hook(void* ped) {            // peringkat 2: fizik penuh
    g_verbose = (g_calls < 3);
    if (g_verbose) { g_calls++; Log("Render dipanggil (penuh) #%d ped=%p", g_calls, ped); }
    if (C.enabled) Apply(ped);
    g_verbose = false;
    Render_orig(ped);
}

extern "C" void OnModLoad() {
    const char* dir = aml->GetConfigPath();
    g_logPath = std::string(dir ? dir : ".") + "/ChestPhysics.log";
    remove(g_logPath.c_str());
    Log("1: OnModLoad bermula, configPath=%s", dir ? dir : "(null)");
    LoadIni();
    Log("2: ini dibaca: stage=%d female_only=%d ped_type_offset=0x%x", C.stage, (int)C.femaleOnly, (unsigned)C.pedTypeOffset);
    void* h = aml->GetLibHandle("libGTASA.so");
    Log("3: handle libGTASA = %p", h);
    if (!h) { Log("GAGAL: libGTASA.so tidak ditemui"); return; }
    GetHier   = (GetHier_t)  aml->GetSym(h, "_Z29GetAnimHierarchyFromSkinClumpP7RpClump");
    GetMatArr = (GetMatArr_t)aml->GetSym(h, "_Z30RpHAnimHierarchyGetMatrixArrayP16RpHAnimHierarchy");
    IdIndex   = (IdIndex_t)  aml->GetSym(h, "_Z17RpHAnimIDGetIndexP16RpHAnimHierarchyi");
    g_timeStep = (float*)    aml->GetSym(h, "_ZN6CTimer12ms_fTimeStepE");
    g_frame    = (uint32_t*) aml->GetSym(h, "_ZN6CTimer14m_FrameCounterE");
    void* render = (void*)aml->GetSym(h, "_ZN4CPed6RenderEv");
    Log("4: simbol: hier=%p mat=%p idx=%p step=%p frame=%p render=%p",
        (void*)GetHier, (void*)GetMatArr, (void*)IdIndex, (void*)g_timeStep, (void*)g_frame, render);
    if (!GetHier || !GetMatArr || !IdIndex || !g_timeStep || !g_frame || !render) {
        Log("GAGAL: satu atau lebih simbol tidak ditemui"); return;
    }
    if (C.stage <= 0) { Log("5: stage 0 - tiada pemasangan (ujian muat sahaja)"); return; }

    // Pasang melalui vtable (bukan hook inline) supaya tidak bertembung dengan mod lain
    // yang meng-hook badan CPed::Render. Slot 21 = CPed::Render (disahkan dari libGTASA.so v2.0).
    const int kRenderSlot = 21;
    void** vt = (void**)aml->GetSym(h, "_ZTV4CPed");
    Log("5: vtable CPed = %p", (void*)vt);
    if (!vt) { Log("GAGAL: vtable CPed tiada"); return; }
    if (vt[kRenderSlot] != render)
        Log("AMARAN: slot %d (%p) bukan CPed::Render (%p) - mungkin dipatch mod lain; guna nilai semasa sebagai asal",
            kRenderSlot, vt[kRenderSlot], render);
    void* origEntry = vt[kRenderSlot];
    Render_orig = (void (*)(void*))origEntry;
    void* repl = (C.stage == 1) ? (void*)Render_pass : (void*)Render_hook;

    static const char* kVtables[] = { "_ZTV4CPed", "_ZTV10CPlayerPed", "_ZTV12CCivilianPed",
                                      "_ZTV7CCopPed", "_ZTV13CEmergencyPed" };
    long pg = sysconf(_SC_PAGESIZE);
    int patched = 0;
    for (const char* name : kVtables) {
        void** t = (void**)aml->GetSym(h, name);
        if (!t) { Log("  %s: tiada", name); continue; }
        if (t[kRenderSlot] != origEntry) { Log("  %s: slot lain (%p), dilangkau", name, t[kRenderSlot]); continue; }
        void** slot = &t[kRenderSlot];
        uintptr_t page = (uintptr_t)slot & ~(uintptr_t)(pg - 1);
        if (mprotect((void*)page, (size_t)pg, PROT_READ | PROT_WRITE) != 0) { Log("  %s: mprotect gagal", name); continue; }
        *slot = repl;
        patched++;
        Log("  %s: dipatch", name);
    }
    Log("6: selesai, %d vtable dipatch (stage %d)", patched, C.stage);
}
