"""
Multi target hardware simulator for missile/defense tracking systems
Generates telemetry data for 3D tracking with sensor faults

Running the sim: py/python ares_hwsim.py
Connect on 127.0.0.1 port 4444 with your tracking client
Simulates up to 6 target types, or more if you add them, with varying kinematics and RCS
Injects track swaps, false targets, packet drops, and latency
Responds to missile launch guidance commands

With love from The SNEK Initiative, muah <3
"""

import socket
import time
import sys
import random
import math

class SimTgt:
    def __init__(self, tgtid, ttype, rng, az, el, rv, snr, lock, rcs, launc):
        self.id = tgtid
        self.type = ttype
        self.range = rng
        self.az = az
        self.el = el
        self.rv = rv
        self.snr = snr
        self.lock = lock
        self.rcs = rcs
        self.launc = launc
        self.ticks = 0
        self.active = True
        self.lauced = False
        
        elrd = (self.el / 1000.0) * (math.pi / 180.0)
        azrd = (self.az / 1000.0) * (math.pi / 180.0)
        self.z = self.range * math.sin(elrd)
        self.x = self.range * math.cos(elrd) * math.cos(azrd)
        self.y = self.range * math.cos(elrd) * math.sin(azrd)
        
        if self.range > 0.1:
            self.vx = (self.x / self.range) * self.rv
            self.vy = (self.y / self.range) * self.rv
            self.vz = (self.z / self.range) * self.rv
        else:
            self.vx, self.vy, self.vz = 0.0, 0.0, 0.0

        self.missla = False
        self.misx = 0.0
        self.misy = 0.0
        self.misz = 0.0
        self.misvx = 0.0
        self.misvy = 0.0
        self.misvz = 0.0
        self.fuel = 100.0
        self.strx = 0.0
        self.stry = 0.0
        self.strz = 0.0
        self.latmps = 0
        self.lcdown = 0.0
        self.srbias = random.gauss(0.0, 12.0)
        self.sazbis = random.gauss(0.0, 45.0)
        self.selbis = random.gauss(0.0, 30.0)
        self.srvbis = random.gauss(0.0, 2.0)
        self.tcstck = 0
        self.pkdrst = 0
        self.crptbs = 0
        self.latspk = 0.0
        self.lstpkt = None
        self.estx = None
        self.esty = None
        self.estz = None
        self.estevx = None
        self.estevy = None
        self.estevz = None

    def lvel(self):
        lspd = 800.0
        ldirx = self.estx if self.estx is not None else self.strx
        ldiry = self.esty if self.esty is not None else self.stry
        ldirz = self.estz if self.estz is not None else self.strz

        lmag = math.sqrt(ldirx**2 + ldiry**2 + ldirz**2)
        if lmag < 1e-6:
            if self.estevx is not None:
                lsec = 1.0
                ldirx = self.estx + self.estevx * lsec
                ldiry = self.esty + self.estevy * lsec
                ldirz = self.estz + self.estevz * lsec
            else:
                lsec = 0.75
                ldirx = self.x + self.vx * lsec
                ldiry = self.y + self.vy * lsec
                ldirz = self.z + self.vz * lsec
            lmag = math.sqrt(ldirx**2 + ldiry**2 + ldirz**2)

        if lmag > 1e-6:
            self.misvx = (ldirx / lmag) * lspd
            self.misvy = (ldiry / lmag) * lspd
            self.misvz = (ldirz / lmag) * lspd
        else:
            self.misvx = lspd
            self.misvy = 0.0
            self.misvz = 0.0

    def lauthz(self):
        if self.lcdown > 0.0:
            return False
        if self.type == 'fast_missile' and self.latmps >= 1:
            return False
        if self.latmps >= 2:
            return False
        if not self.lock:
            return False
        if self.rv >= -5.0:
            return False
        if self.range > 65000.0:
            return False
        if self.launc > 90000:
            return False
        if self.type == 'decoy' and self.range > 22000.0:
            return False
        if self.type == 'fast_missile' and self.range > 36000.0:
            return False
        if self.type == 'hypersonic' and self.range > 50000.0:
            return False
        return True

    def upsnrf(self):
        self.srbias = self.srbias * 0.98 + random.gauss(0.0, 1.8)
        self.sazbis = self.sazbis * 0.97 + random.gauss(0.0, 7.5)
        self.selbis = self.selbis * 0.97 + random.gauss(0.0, 5.0)
        self.srvbis = self.srvbis * 0.98 + random.gauss(0.0, 0.6)

        if self.tcstck > 0:
            self.tcstck -= 1
        if self.pkdrst > 0:
            self.pkdrst -= 1
        if self.crptbs > 0:
            self.crptbs -= 1
        if self.latspk > 0.0:
            self.latspk = max(0.0, self.latspk - 90.0)

        if self.lock and random.random() < 0.004:
            self.tcstck = max(self.tcstck, random.randint(2, 6))
        if random.random() < 0.002:
            self.pkdrst = max(self.pkdrst, random.randint(1, 4))
        if random.random() < 0.0015:
            self.crptbs = max(self.crptbs, random.randint(2, 5))
        if random.random() < 0.001:
            self.latspk = max(self.latspk, random.uniform(120.0, 800.0))

    def blspkt(self, pktid=None, src=None, stale=False):
        src = src or self
        pktid = self.id if pktid is None else pktid

        if self.pkdrst > 0:
            return None
        if random.random() < (0.01 if src.type == 'decoy' else 0.004):
            return None

        if self.latspk > 0.0:
            time.sleep(self.latspk / 1000.0)

        if (stale or self.tcstck > 0) and self.lstpkt is not None and random.random() < 0.9:
            return self.lstpkt

        lock = src.lock
        if src.range > 65000.0 or src.snr < 8:
            lock = 0
        nrng = max(0.0, src.range + random.gauss(0.0, max(35.0, src.range * 0.0012)) + self.srbias)
        naz = src.az + random.gauss(0.0, 70.0 if src.type != 'decoy' else 95.0) + self.sazbis
        nel = src.el + random.gauss(0.0, 50.0 if src.type != 'decoy' else 70.0) + self.selbis
        nrv = src.rv + random.gauss(0.0, 4.0 if src.type != 'hypersonic' else 7.0) + self.srvbis
        nsnr = max(0, min(99, int(src.snr + random.randint(-3, 3))))

        cmode = stale or self.tcstck > 0 or lock == 0
        if cmode:
            cscal = 1.0 + 0.35 * max(self.tcstck, 1)
            nrng += random.gauss(0.0, 80.0 * cscal)
            naz += random.gauss(0.0, 160.0 * cscal)
            nel += random.gauss(0.0, 110.0 * cscal)
            nrv += random.gauss(0.0, 8.0 * cscal)
            nsnr = max(0, nsnr - random.randint(4, 12))
            lock = 0

        if self.crptbs > 0 or random.random() < (0.01 if src.type in ('fast_missile', 'hypersonic') else 0.003):
            nrng *= random.uniform(0.6, 1.6)
            naz += random.uniform(-180000.0, 180000.0)
            nel += random.uniform(-120000.0, 120000.0)
            nrv += random.uniform(-750.0, 750.0)
            nsnr = max(0, nsnr - random.randint(6, 25))
            lock = random.choice([0, lock, 1])

        eestd = (nel / 1000.0) * (math.pi / 180.0)
        aestd = (naz / 1000.0) * (math.pi / 180.0)
        self.estx = nrng * math.cos(eestd) * math.cos(aestd)
        self.esty = nrng * math.cos(eestd) * math.sin(aestd)
        self.estz = nrng * math.sin(eestd)
        if nrng > 0.1:
            self.estevx = (self.estx / nrng) * nrv
            self.estevy = (self.esty / nrng) * nrv
            self.estevz = (self.estz / nrng) * nrv
        else:
            self.estevx = 0.0
            self.estevy = 0.0
            self.estevz = 0.0

        pkt = (f"TELEMETRY {pktid} {nrng:.2f} {naz:.2f} {nel:.2f} "
                  f"{nrv:.2f} {nsnr} {lock} {src.rcs} {src.launc} "
                  f"{src.misx:.2f} {src.misy:.2f} {src.misz:.2f} "
                  f"{src.misvx:.2f} {src.misvy:.2f} {src.misvz:.2f} "
                  f"{src.fuel:.2f}\n")
        self.lstpkt = pkt
        return pkt

    def upd(self):
        self.ticks += 1
        dt = 0.1
        
        if self.type == 'decoy':
            self.vx += random.uniform(-1.0, 1.0)
            self.vy += random.uniform(-1.0, 1.0)
            self.vz += random.uniform(-0.5, 0.5)
            if self.ticks % 18 == 0:
                self.vx += random.uniform(-40.0, 40.0)
                self.vy += random.uniform(-40.0, 40.0)
                self.snr = max(5, self.snr - random.randint(1, 4))
        elif self.type == 'fast_missile' and self.ticks == 25:
            print(f"\n[SIM EVENT] Target {self.id} performs sudden highG evasive maneuver")
            self.vx += 300.0
            self.vy -= 300.0
            self.rcs = 40
        elif self.type == 'hypersonic' and self.ticks % 30 == 0:
            print(f"\n[SIM EVENT] Target {self.id} performs evasive lateral deviation!")
            self.vx += random.uniform(-180.0, 180.0)
            self.vy += random.uniform(-180.0, 180.0)
            self.snr = max(6, self.snr - random.randint(0, 3))
            
        self.x += self.vx * dt
        self.y += self.vy * dt
        self.z += self.vz * dt
        
        self.range = math.sqrt(self.x**2 + self.y**2 + self.z**2)
        if self.range > 0.1:
            elrd = math.asin(self.z / self.range)
            self.el = (elrd * 180.0 / math.pi) * 1000.0
            
            azrd = math.atan2(self.y, self.x)
            self.az = (azrd * 180.0 / math.pi) * 1000.0
            
            self.rv = (self.vx * self.x + self.vy * self.y + self.vz * self.z) / self.range
        else:
            self.range = 0.0
            self.el = 0.0
            self.az = 0.0
            self.rv = 0.0

        if self.lcdown > 0.0:
            self.lcdown = max(0.0, self.lcdown - dt)

        if self.missla:
            mdry = 150.0
            mtotal = mdry + self.fuel
            
            thrsx, thrsy, thrsz = 0.0, 0.0, 0.0
            if self.fuel > 0.0:
                self.fuel -= 2.0 * dt
                if self.fuel < 0.0:
                    self.fuel = 0.0
                
                fthrs = 50000.0
                vmag = math.sqrt(self.misvx**2 + self.misvy**2 + self.misvz**2)
                if vmag > 0.1:
                    thrsx = (self.misvx / vmag) * fthrs
                    thrsy = (self.misvy / vmag) * fthrs
                    thrsz = (self.misvz / vmag) * fthrs
                else:
                    thrsz = fthrs
            
            vmag = math.sqrt(self.misvx**2 + self.misvy**2 + self.misvz**2)
            dragx, dragy, dragz = 0.0, 0.0, 0.0
            if vmag > 0.1:
                dragf = 0.5 * 1.2 * 0.12 * vmag / mtotal
                dragx = -self.misvx * dragf
                dragy = -self.misvy * dragf
                dragz = -self.misvz * dragf
                
            smag = math.sqrt(self.strx**2 + self.stry**2 + self.strz**2)
            mxstr = 250.0
            sx, sy, sz = self.strx, self.stry, self.strz
            if smag > mxstr:
                sx = (sx / smag) * mxstr
                sy = (sy / smag) * mxstr
                sz = (sz / smag) * mxstr
                
            ax = (thrsx / mtotal) + dragx + sx
            ay = (thrsy / mtotal) + dragy + sy
            az = (thrsz / mtotal) + dragz + sz - 9.81
            
            self.misvx += ax * dt
            self.misvy += ay * dt
            self.misvz += az * dt
            
            self.misx += self.misvx * dt
            self.misy += self.misvy * dt
            self.misz += self.misvz * dt
            
            dist = math.sqrt((self.x - self.misx)**2 + (self.y - self.misy)**2 + (self.z - self.misz)**2)
            if dist <= 15.0:
                self.active = False
                print(f"\n[SIM EVENT] Target {self.id} INTERCEPTED and DESTROYED! Direct hit at 3D range {int(dist)}m!")
                return
                
            if self.misz < 0.0 or (self.fuel <= 0.0 and vmag < 30.0) or dist > 120000.0:
                print(f"\n[SIM EVENT] Missile launch against target {self.id} FAILED")
                self.missla = False
                self.lauced = False
                self.latmps += 1
                self.lcdown = 8.0

        if self.range < 1500.0:
            self.active = False
            print(f"\n[SIM EVENT] Target {self.id} IMPACTED BASE")

def compai(tgt):
    mposx = tgt.misx
    mposy = tgt.misy
    mposz = tgt.misz
    mvelx = tgt.misvx
    mvely = tgt.misvy
    mvelz = tgt.misvz

    rposx = tgt.x - mposx
    rposy = tgt.y - mposy
    rposz = tgt.z - mposz
    rvelx = tgt.vx - mvelx
    rvely = tgt.vy - mvely
    rvelz = tgt.vz - mvelz

    dist = math.sqrt(rposx**2 + rposy**2 + rposz**2)
    if dist < 1e-6:
        return 0.0, 0.0, 0.0

    lsec = max(0.6, min(5.0, dist / 1100.0))
    aimx = tgt.x + tgt.vx * lsec
    aimy = tgt.y + tgt.vy * lsec
    aimz = tgt.z + tgt.vz * lsec

    cmdx = (aimx - mposx) + (rvelx * 0.35)
    cmdy = (aimy - mposy) + (rvely * 0.35)
    cmdz = (aimz - mposz) + (rvelz * 0.35)

    cmag = math.sqrt(cmdx**2 + cmdy**2 + cmdz**2)
    if cmag < 1e-6:
        cmdx, cmdy, cmdz = rposx, rposy, rposz
        cmag = dist

    if cmag > 325.0:
        scl = 325.0 / cmag
        cmdx *= scl
        cmdy *= scl
        cmdz *= scl

    return cmdx, cmdy, cmdz

def allid(atgts, ftcks):
    useid = set(atgts.keys()) | {trk["id"] for trk in ftcks}
    for cand in range(31, 2, -1):
        if cand not in useid:
            return cand
    return None

def spftrk(atgts, ftcks, ticks):
    fkid = allid(atgts, ftcks)
    if fkid is None:
        return

    brng = random.uniform(60000.0, 110000.0)
    baz = random.uniform(5000.0, 22000.0)
    bel = random.uniform(4000.0, 26000.0)
    brv = random.uniform(-900.0, -30.0)
    ftcks.append({
        "id": fkid,
        "range": brng,
        "az": baz,
        "el": bel,
        "rv": brv,
        "snr": random.randint(6, 28),
        "lock": random.choice([0, 0, 1]),
        "rcs": random.randint(10, 2000),
        "launc": random.choice([95000, 99000]),
        "phase": "live",
        "tleft": random.randint(2, 5),
        "cleft": random.randint(1, 3),
        "rngbs": random.gauss(0.0, 50.0),
        "azbs": random.gauss(0.0, 120.0),
        "elbs": random.gauss(0.0, 90.0),
        "rvbs": random.gauss(0.0, 10.0),
        "latms": 0.0,
        "dburp": 0,
        "cburp": 0,
        "tcrt": ticks,
    })

def bfpkt(trk):
    if trk["tleft"] <= 0:
        return None

    trk["rngbs"] = trk["rngbs"] * 0.98 + random.gauss(0.0, 4.0)
    trk["azbs"] = trk["azbs"] * 0.97 + random.gauss(0.0, 9.0)
    trk["elbs"] = trk["elbs"] * 0.97 + random.gauss(0.0, 7.0)
    trk["rvbs"] = trk["rvbs"] * 0.98 + random.gauss(0.0, 1.2)

    if trk["phase"] == "live":
        trk["range"] += random.uniform(-250.0, 180.0)
        trk["az"] += random.uniform(-220.0, 220.0)
        trk["el"] += random.uniform(-160.0, 160.0)
        trk["rv"] += random.uniform(-35.0, 35.0)
        trk["snr"] = max(5, min(95, trk["snr"] + random.randint(-2, 2)))
        if random.random() < 0.08:
            trk["dburp"] = max(trk["dburp"], random.randint(1, 3))
        if random.random() < 0.06:
            trk["cburp"] = max(trk["cburp"], random.randint(1, 4))
        if random.random() < 0.03:
            trk["latms"] = max(trk["latms"], random.uniform(120.0, 700.0))
        trk["tleft"] -= 1
        if trk["tleft"] <= trk["cleft"]:
            trk["phase"] = "coast"
    else:
        trk["range"] += random.uniform(-60.0, 20.0)
        trk["az"] += random.uniform(-50.0, 50.0)
        trk["el"] += random.uniform(-40.0, 40.0)
        trk["rv"] += random.uniform(-15.0, 15.0)
        trk["snr"] = max(0, trk["snr"] - random.randint(1, 5))
        trk["lock"] = 0
        trk["cleft"] -= 1
        if trk["cleft"] <= 0:
            trk["tleft"] = 0

    if trk["dburp"] > 0:
        trk["dburp"] -= 1
        return None
    if random.random() < 0.04 and trk["phase"] == "coast":
        return None

    if trk["latms"] > 0.0:
        time.sleep(trk["latms"] / 1000.0)
        trk["latms"] = max(0.0, trk["latms"] - 90.0)

    nrng = max(0.0, trk["range"] + random.gauss(0.0, max(30.0, trk["range"] * 0.0015)) + trk["rngbs"])
    naz = trk["az"] + random.gauss(0.0, 70.0) + trk["azbs"]
    nel = trk["el"] + random.gauss(0.0, 55.0) + trk["elbs"]
    nrv = trk["rv"] + random.gauss(0.0, 5.0) + trk["rvbs"]
    nsnr = max(0, min(99, int(trk["snr"] + random.randint(-3, 3))))
    lock = trk["lock"] if trk["phase"] == "live" else 0

    if trk["cburp"] > 0:
        trk["cburp"] -= 1
        nrng *= random.uniform(0.5, 1.8)
        naz += random.uniform(-180000.0, 180000.0)
        nel += random.uniform(-120000.0, 120000.0)
        nrv += random.uniform(-600.0, 600.0)
        nsnr = max(0, nsnr - random.randint(8, 22))
        lock = random.choice([0, lock, 1])

    return (f"TELEMETRY {trk['id']} {nrng:.2f} {naz:.2f} {nel:.2f} "
            f"{nrv:.2f} {nsnr} {lock} {trk['rcs']} {trk['launc']} "
            f"0.00 0.00 0.00 0.00 0.00 0.00\n")

def main():
    ssock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    ssock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    
    try:
        ssock.bind(("127.0.0.1", 4444))
    except Exception as e:
        print(f"[SIM ERROR] Failed to bind to port 4444: {e}")
        sys.exit(1)
        
    ssock.listen(1)
    
    while True:
        print("\n[SIM] Listening on tcp://127.0.0.1:4444...")
        
        try:
            conn, addr = ssock.accept()
            print(f"[SIM] Connected by {addr}")
        except KeyboardInterrupt:
            print("\n[SIM] Exiting.")
            break
            
        tgts = {}
        tgts[0] = SimTgt(0, 'decoy', 50000.0, 12000.0, 20000.0, 10.0, 30, 1, 5000, 5000)
        tgts[1] = SimTgt(1, 'closing', 60000.0, 14000.0, 20000.0, -250.0, 80, 1, 5000, 3500)
        tgts[2] = SimTgt(2, 'fast_missile', 40000.0, 10000.0, 600.0, -600.0, 92, 1, 150, 2000)
        ntgtid = 3
        ftcks = []
        
        ticks = 0
        sbuf = ""
        pguid = {}

        def rdguid(tgtid, tout=0.15):
            nonlocal sbuf
            ddline = time.time() + tout
            while time.time() < ddline:
                cchd = pguid.pop(tgtid, None)
                if cchd is not None:
                    return cchd

                try:
                    data = conn.recv(1024).decode('utf-8', errors='ignore')
                    if not data:
                        break
                    sbuf += data
                except socket.timeout:
                    pass

                while '\n' in sbuf:
                    line, sbuf = sbuf.split('\n', 1)
                    line = line.strip()
                    if not line.startswith("GUIDANCE"):
                        continue

                    pts = line.split()
                    if len(pts) < 6:
                        continue

                    try:
                        ptgtid = int(pts[1])
                        pguid[ptgtid] = (
                            float(pts[2]),
                            float(pts[3]),
                            float(pts[4]),
                            int(pts[5]),
                        )
                    except ValueError:
                        continue

                cchd = pguid.pop(tgtid, None)
                if cchd is not None:
                    return cchd

            return None
        
        try:
            conn.settimeout(None)
            print("[SIM] Waiting for START_AUTOPILOT signal")
            
            sigbf = ""
            while "START_AUTOPILOT" not in sigbf:
                data = conn.recv(1024).decode('utf-8', errors='ignore')
                if not data:
                    raise socket.error("Connection closed while waiting for START_AUTOPILOT")
                sigbf += data
                
            print("[SIM] START_AUTOPILOT signal received")
            conn.settimeout(0.05)
            
            while True:
                ticks += 1
                
                for tid, tgt in list(tgts.items()):
                    if not tgt.active:
                        del tgts[tid]
                
                if ticks % 30 == 0 and len(tgts) < 5:
                    ttype = random.choices(
                        ['decoy', 'closing', 'fast_missile', 'hypersonic', 'bomber', 'drone'],
                        weights=[25, 35, 15, 10, 10, 5],
                        k=1,
                    )[0]
                    if ttype == 'decoy':
                        trng = random.uniform(40000.0, 60000.0)
                        taz = random.uniform(8000.0, 20000.0)
                        tel = random.uniform(15000.0, 25000.0)
                        trv = random.uniform(50.0, 150.0)
                        tsnr = random.randint(25, 45)
                        trcs = 5000
                        tlnc = 5000
                    elif ttype == 'closing':
                        trng = random.uniform(50000.0, 70000.0)
                        taz = random.uniform(8000.0, 20000.0)
                        tel = random.uniform(15000.0, 25000.0)
                        trv = random.uniform(-300.0, -200.0)
                        tsnr = random.randint(75, 85)
                        trcs = 5000
                        tlnc = 3500
                    elif ttype == 'fast_missile':
                        trng = random.uniform(35000.0, 45000.0)
                        taz = random.uniform(8000.0, 12000.0)
                        tel = random.uniform(500.0, 800.0)
                        trv = random.uniform(-650.0, -550.0)
                        tsnr = random.randint(88, 95)
                        trcs = 150
                        tlnc = 2000
                    elif ttype == 'hypersonic':
                        trng = random.uniform(80000.0, 100000.0)
                        taz = random.uniform(15000.0, 25000.0)
                        tel = random.uniform(25000.0, 35000.0)
                        trv = random.uniform(-1900.0, -1500.0)
                        tsnr = random.randint(70, 80)
                        trcs = 1500
                        tlnc = 1500
                    elif ttype == 'bomber':
                        trng = random.uniform(70000.0, 90000.0)
                        taz = random.uniform(10000.0, 20000.0)
                        tel = random.uniform(15000.0, 25000.0)
                        trv = random.uniform(-260.0, -200.0)
                        tsnr = random.randint(85, 95)
                        trcs = 200000
                        tlnc = 4000
                    elif ttype == 'drone':
                        trng = random.uniform(1500.0, 3000.0)
                        taz = random.uniform(10000.0, 20000.0)
                        tel = random.uniform(8000.0, 15000.0)
                        trv = random.uniform(-40.0, -20.0)
                        tsnr = random.randint(55, 65)
                        trcs = 500
                        tlnc = 1000
                    
                    tgts[ntgtid] = SimTgt(
                        ntgtid, ttype, trng, taz, tel, trv, tsnr, 1, trcs, tlnc
                    )
                    print(f"\n[SIM EVENT] New target detected: ID {ntgtid} at range {int(trng)}m")
                    ntgtid += 1
                
                if not tgts:
                    tgts[0] = SimTgt(0, 'decoy', 50000.0, 12000.0, 20000.0, 10.0, 30, 1, 5000, 5000)
                    print("\n[SIM EVENT] Re-spawned baseline target")

                if len(ftcks) < 1 and random.random() < 0.006:
                    spftrk(tgts, ftcks, ticks)
                
                for tid, tgt in list(tgts.items()):
                    tgt.upd()
                    tgt.upsnrf()
                    if not tgt.active:
                        continue

                    swpsrc = None
                    if len(tgts) > 1 and random.random() < 0.004 and (tgt.tcstck > 0 or tgt.snr < 20):
                        swapc = [othr for othr in tgts.values() if othr.id != tgt.id and othr.active]
                        if swapc:
                            swpsrc = random.choice(swapc)
                            print(f"[SIM WARN] Track swap: sending Target {swpsrc.id} truth under Track {tgt.id}")

                    pkt = tgt.blspkt(src=swpsrc or tgt)
                    if pkt is None:
                        if tgt.tcstck > 0 or tgt.pkdrst > 0:
                            print(f"[SIM WARN] Track {tgt.id} packet drop")
                        continue

                    conn.sendall(pkt.encode('utf-8'))

                    acx, acy, acz = tgt.strx, tgt.stry, tgt.strz
                    res = rdguid(tgt.id)
                    if res is not None:
                        acx, acy, acz, lflag = res
                        tgt.strx = acx
                        tgt.stry = acy
                        tgt.strz = acz

                        if lflag == 1 and not tgt.lauced and tgt.lauthz():
                            tgt.lauced = True
                            tgt.missla = True
                            tgt.fuel = 100.0
                            tgt.misx = 0.0
                            tgt.misy = 0.0
                            tgt.misz = 0.0
                            tgt.lvel()
                            print(f"[SIM EVENT] Guidance system engaged! Missile LAUNCHED")

                    if tgt.lauced:
                        print(f"[SIM] Target {tgt.id} Missile Steering Accel: ({acx:.1f}, {acy:.1f}, {acz:.1f}) Fuel: {tgt.fuel:.1f}kg")

                for trk in list(ftcks):
                    pkt = bfpkt(trk)
                    if pkt is None:
                        if trk["tleft"] <= 0:
                            ftcks.remove(trk)
                        continue

                    conn.sendall(pkt.encode('utf-8'))
                    fres = rdguid(trk["id"])
                    if fres is not None:
                        _, _, _, lflag = fres
                        if lflag == 1:
                            print(f"[SIM WARN] False track {trk['id']} triggered a launch request")
                    if trk["tleft"] <= 0 and trk["cleft"] <= 0:
                        ftcks.remove(trk)
                                    
                time.sleep(0.1)
                
        except (socket.error, ConnectionResetError) as e:
            print(f"[SIM] Client disconnected: {e}")
        finally:
            conn.close()

if __name__ == "__main__":
    main()
