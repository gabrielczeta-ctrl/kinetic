"""Symbolize a Windows minidump's faulting frame with dbghelp + the build's PDB.
usage: python tools/body/dumpsym.py DUMP.dmp path/to/openmw.exe (2026-10-06)."""
import ctypes, ctypes.wintypes as wt, mmap, struct, sys, os
dump = sys.argv[1]; exe = sys.argv[2]
dbghelp = ctypes.WinDLL('dbghelp')
f = open(dump, 'rb'); data = f.read()
buf = ctypes.create_string_buffer(data, len(data))
base = ctypes.addressof(buf)
class DIR(ctypes.Structure): _fields_=[('StreamType',wt.ULONG),('DataSize',wt.ULONG),('Rva',wt.ULONG)]
dbghelp.MiniDumpReadDumpStream.argtypes=[ctypes.c_void_p, wt.ULONG, ctypes.POINTER(ctypes.POINTER(DIR)), ctypes.POINTER(ctypes.c_void_p), ctypes.POINTER(wt.ULONG)]
dbghelp.MiniDumpReadDumpStream.restype=wt.BOOL
def stream(t):
    d=ctypes.POINTER(DIR)(); p=ctypes.c_void_p(); n=wt.ULONG()
    if not dbghelp.MiniDumpReadDumpStream(base, t, ctypes.byref(d), ctypes.byref(p), ctypes.byref(n)): return None
    return p.value - base, n.value
off,_ = stream(6)  # ExceptionStream
tid, _al = struct.unpack_from('<II', data, off)
code, flags, rec, addr, nparams = struct.unpack_from('<IIQQI', data, off+8)
ctx_size, ctx_rva = struct.unpack_from('<II', data, off+8+152+0) if False else (0,0)
print('thread', tid, 'code %08x addr %x' % (code, addr))
# module list
moff,_ = stream(4)
n = struct.unpack_from('<I', data, moff)[0]
mods=[]
for i in range(n):
    o = moff+4+i*108
    mbase, msize = struct.unpack_from('<QI', data, o)
    nrva = struct.unpack_from('<I', data, o+20)[0]
    ln = struct.unpack_from('<I', data, nrva)[0]
    name = data[nrva+4:nrva+4+ln].decode('utf-16-le')
    mods.append((mbase, msize, name))
def modof(a):
    for b,s,nm in mods:
        if b <= a < b+s: return b,s,nm
    return None
print('fault module', modof(addr))
# exception thread context: find in thread list
toff,_ = stream(3)
nt = struct.unpack_from('<I', data, toff)[0]
ctx=None; stack=None
for i in range(nt):
    o = toff+4+i*48
    t_id = struct.unpack_from('<I', data, o)[0]
    st_start, st_size, st_rva = struct.unpack_from('<QII', data, o+24)
    c_size, c_rva = struct.unpack_from('<II', data, o+40)
    if t_id == tid: ctx=(c_size,c_rva); stack=(st_start, st_size, st_rva)
# use exception stream's context instead (more accurate)
ec_size, ec_rva = struct.unpack_from('<II', data, off+8+152)
ctxb = data[ec_rva:ec_rva+ec_size]
rsp = struct.unpack_from('<Q', ctxb, 0x98)[0]; rip = struct.unpack_from('<Q', ctxb, 0xF8)[0]
print('rip %x rsp %x' % (rip, rsp))
# symbols
proc = wt.HANDLE(0x1234)
dbghelp.SymSetOptions(0x2 | 0x10 | 0x4)  # undname, load lines, deferred
dbghelp.SymInitializeW.argtypes=[wt.HANDLE, wt.LPCWSTR, wt.BOOL]
dbghelp.SymInitializeW(proc, os.path.dirname(exe), False)
dbghelp.SymLoadModuleExW.argtypes=[wt.HANDLE, wt.HANDLE, wt.LPCWSTR, wt.LPCWSTR, ctypes.c_uint64, wt.DWORD, ctypes.c_void_p, wt.DWORD]
dbghelp.SymLoadModuleExW.restype=ctypes.c_uint64
for b,s,nm in mods:
    if nm.lower().endswith('openmw.exe'):
        r = dbghelp.SymLoadModuleExW(proc, None, exe, None, b, s, None, 0); print('loaded openmw', hex(b), r!=0)
class SYM(ctypes.Structure):
    _fields_=[('SizeOfStruct',wt.ULONG),('TypeIndex',wt.ULONG),('Reserved',ctypes.c_uint64*2),('Index',wt.ULONG),('Size',wt.ULONG),('ModBase',ctypes.c_uint64),('Flags',wt.ULONG),('Value',ctypes.c_uint64),('Address',ctypes.c_uint64),('Register',wt.ULONG),('Scope',wt.ULONG),('Tag',wt.ULONG),('NameLen',wt.ULONG),('MaxNameLen',wt.ULONG),('Name',ctypes.c_char*1024)]
class LINE(ctypes.Structure):
    _fields_=[('SizeOfStruct',wt.DWORD),('Key',ctypes.c_void_p),('LineNumber',wt.DWORD),('FileName',ctypes.c_char_p),('Address',ctypes.c_uint64)]
def sym(a):
    s=SYM(); s.SizeOfStruct=ctypes.sizeof(SYM)-1024; s.MaxNameLen=1000
    disp=ctypes.c_uint64()
    ok=dbghelp.SymFromAddr(proc, ctypes.c_uint64(a), ctypes.byref(disp), ctypes.byref(s))
    name = s.Name.decode(errors='replace') if ok else '?'
    ln=LINE(); ln.SizeOfStruct=ctypes.sizeof(LINE); d2=wt.DWORD()
    okl=dbghelp.SymGetLineFromAddr64(proc, ctypes.c_uint64(a), ctypes.byref(d2), ctypes.byref(ln))
    loc = '%s:%d' % (os.path.basename(ln.FileName.decode()), ln.LineNumber) if okl else ''
    return name, loc
print('FAULT', sym(rip))
# scan stack memory for return addresses inside openmw.exe (heuristic backtrace)
ranges=[]
m64 = stream(9)  # Memory64ListStream (full dumps)
if m64:
    nr, baserva = struct.unpack_from('<QQ', m64[0] and data, m64[0]); cur=baserva
    for i in range(nr):
        st, sz = struct.unpack_from('<QQ', data, m64[0]+16+i*16)
        ranges.append((st,sz,cur)); cur+=sz
else:
    m32 = stream(5)  # MemoryListStream (minidumps: thread stacks)
    nr = struct.unpack_from('<I', data, m32[0])[0]
    for i in range(nr):
        st, sz, rva = struct.unpack_from('<QII', data, m32[0]+4+i*16)
        ranges.append((st,sz,rva))
def readmem(a, n):
    for st,sz,r in ranges:
        if st<=a and a+n<=st+sz: return data[r+(a-st):r+(a-st)+n]
    return None
ob = [m for m in mods if m[2].lower().endswith('openmw.exe')][0]
seen=0
for k in range(0, 0x3000, 8):
    v = readmem(rsp+k, 8)
    if v is None: break
    a = struct.unpack('<Q', v)[0]
    if ob[0] <= a < ob[0]+ob[1]:
        n_, l_ = sym(a)
        if n_ != '?':
            print('  +%04x %s %s' % (k, n_[:110], l_)); seen+=1
            if seen>=40: break
