#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
sensors.py — проверка и калибровка датчиков тележки на ноутбуке (Linux).
Работает с прошивкой Telega5 (ESP32, USB, 115200; команды датчиков — с префиксом N): на каждой тележке HC-SR04, GY-521
и два дальномера CJMCU-531 (VL53L1X): H — высота над головкой рельса, G — расстояние до рабочей грани (колея).
Тележка A (левая, с роликами) и тележка B (правая). Сторона выбирается ключом --side A|B.

Установка:  pip install pyserial
Режимы:
  ports                                  — список последовательных портов
  monitor     --port /dev/ttyUSB0        — живые показания всех датчиков
  cmd         --port ... "N S500"        — отправить команду (датчики — с префиксом N, движение — без; см. README Telega5)
  us-calib    --port ... --side A        — калибровка ультразвука по рулетке + проверка «пустого» рельса
  imu-calib   --port ...                 — смещения обоих IMU в покое, крен/тангаж установки, шум
  imu-steps   --port ... --side A [--calib]  — запись проездов через стык; --calib: пороги по пластинам 2 и 4 мм
  tof-check   --port ...                 — 10 с покоя: доля годных отсчётов и шум всех 4 дальномеров
  tof-height-calib --port ... --side A [--plates 0,1,2,4,5]  — нуль и масштаб дальномера высоты по пластинам
  tof-gauge-calib  --port ... [--block 10]  — привязка дальномеров колеи к путевому шаблону (сумма A + B)
  tof-live    --port ... [--thr 1,2,4,5] [--out tof_log.csv]  — живое измерение: ступенька в стыке и ширина колеи
Результаты калибровок — в sensors_config.yaml рядом со скриптом.
"""
import argparse, math, os, statistics, sys, time

CFG = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'sensors_config.yaml')


# ------------------------------------------------------------------ конфиг
def cfg_load():
    d = {}
    if os.path.exists(CFG):
        for ln in open(CFG, encoding='utf-8'):
            ln = ln.split('#')[0].strip()
            if ':' in ln:
                k, v = ln.split(':', 1)
                try:
                    d[k.strip()] = float(v)
                except ValueError:
                    d[k.strip()] = v.strip()
    return d


def cfg_save(upd):
    d = cfg_load(); d.update(upd)
    with open(CFG, 'w', encoding='utf-8') as f:
        f.write(f'# sensors.py, обновлено {time.strftime("%Y-%m-%d %H:%M")}\n')
        for k in sorted(d):
            f.write(f'{k}: {d[k]}\n')
    print(f'Сохранено в {CFG}: ' + ', '.join(f'{k}={v}' for k, v in upd.items()))


# ------------------------------------------------------------------ связь с узлом
def parse(line):
    """строка узла -> (тип, словарь) или None; у I, U, E, T есть поле side = 'A' | 'B'"""
    p = line.strip().split(',')
    try:
        if p[0] == 'I' and len(p) == 10:
            k = ('ax', 'ay', 'az', 'gx', 'gy', 'gz', 'pk')
            d = dict(zip(k, [int(x) for x in p[3:]]))
            d['t'] = int(p[1]); d['side'] = p[2]
            for g in ('gx', 'gy', 'gz'):
                d[g] = d[g] / 100.0          # сотые °/с -> °/с
            return 'I', d
        if p[0] == 'U' and len(p) == 5:
            return 'U', {'t': int(p[1]), 'side': p[2], 'raw': int(p[3]), 'med': int(p[4])}
        if p[0] == 'E' and len(p) == 5:
            return 'E', {'t': int(p[1]), 'side': p[2], 'ev': p[3], 'mm': int(p[4])}
        if p[0] == 'T' and len(p) == 6:
            return 'T', {'t': int(p[1]), 'side': p[2], 'kind': p[3], 'raw': int(p[4]), 'st': int(p[5])}
    except ValueError:
        return None
    if line.startswith('#'):
        return '#', {'text': line[1:].strip()}
    return None


class Node:
    def __init__(self, port, baud=115200):
        import serial
        self.s = serial.Serial(port, baud, timeout=0.05)
        time.sleep(2.0)                     # Nano/Uno перезагружается при открытии порта
        self.s.reset_input_buffer()
        self.buf = b''

    def send(self, cmd):
        self.s.write((cmd.strip() + '\n').encode())

    def lines(self, seconds=None):
        t0 = time.time()
        while seconds is None or time.time() - t0 < seconds:
            self.buf += self.s.read(4096)
            while b'\n' in self.buf:
                ln, self.buf = self.buf.split(b'\n', 1)
                r = parse(ln.decode('utf-8', 'replace'))
                if r:
                    yield r

    def collect(self, kind, seconds, side=None):
        return [d for k, d in self.lines(seconds) if k == kind and (side is None or d.get('side') == side)]

    def wait_text(self, needle, seconds=5):
        for k, d in self.lines(seconds):
            if k == '#':
                print('  узел:', d['text'])
                if needle in d['text']:
                    return True
        return False


def roll_pitch(ax, ay, az):
    roll = math.degrees(math.atan2(ay, az))
    pitch = math.degrees(math.atan2(-ax, math.hypot(ay, az)))
    return roll, pitch


# ------------------------------------------------------------------ режимы
def m_ports(a):
    from serial.tools import list_ports
    for p in list_ports.comports():
        print(p.device, '-', p.description)


def m_cmd(a):
    n = Node(a.port)
    n.send(a.text)
    for k, d in n.lines(2):
        if k in ('#', 'E'):
            print(k, d)


def m_monitor(a):
    n = Node(a.port)
    c = cfg_load()
    last = {}; pkwin = {'A': [], 'B': []}; t_print = 0
    print('Ctrl+C — выход. Строка: УЗ (мм) | крен/тангаж (°) | пик |az-1g| за 1 с (мг) | высота H (мм от нуля) — для A и B; колея')
    try:
        for k, d in n.lines():
            if k == '#':
                print('\nузел:', d['text'])
            elif k == 'E':
                if d['side'] == 'M':
                    print('\n*** движение: ' + ('остановлено препятствием' if d['ev'] == 'HOLD' else 'продолжено'))
                else:
                    print(f'\n*** {d["side"]}: {d["ev"]} на {d["mm"]} мм' + ('  (снять: команда N R)' if d['ev'] == 'LOST' else ''))
            elif k == 'T':
                if d['st'] == 0:
                    last[('T' + d['kind'], d['side'])] = d
            else:
                last[(k, d['side'])] = d
                if k == 'I':
                    pkwin[d['side']] = (pkwin[d['side']] + [d['pk']])[-50:]
            if time.time() - t_print > 0.3:
                t_print = time.time()
                out = []
                for sd in ('A', 'B'):
                    u = last.get(('U', sd), {'med': -1}); i = last.get(('I', sd))
                    ua, ub = c.get(f'us_a_{sd}', 1.0), c.get(f'us_b_{sd}', 0.0)
                    dist = u['med'] * ua + ub if u['med'] > 0 else -1
                    if i:
                        rl, pt = roll_pitch(i['ax'], i['ay'], i['az'])
                        im = f'{rl - c.get(f"imu_roll0_{sd}", 0.0):+5.1f}/{pt - c.get(f"imu_pitch0_{sd}", 0.0):+5.1f}° пик {max(pkwin[sd]):4d}'
                    else:
                        im = 'IMU нет данных'
                    h = last.get(('TH', sd))
                    hs = f'{tof_h(c, sd, h["raw"]):+5.1f}' if h else '  нет'
                    out.append(f'{sd}: УЗ {dist:5.0f} | {im} | H {hs}')
                ga, gb = last.get(('TG', 'A')), last.get(('TG', 'B'))
                gs = f'{tof_gauge(c, ga["raw"], gb["raw"]):6.1f}' if ga and gb else ' нет'
                sys.stdout.write('\r' + '  ||  '.join(out) + f'  ||  колея {gs}  '); sys.stdout.flush()
    except KeyboardInterrupt:
        print()


def m_us_calib(a):
    n = Node(a.port); sd = a.side
    n.send('N Q0')                                   # без потока IMU — чище вывод
    refs = [int(x) for x in a.refs.split(',')]
    meas = []
    print(f'Датчик тележки {sd}. Плоская мишень (фанера ~300x300) стоит на головке рельса {sd}, перпендикулярно оси датчика.')
    for r in refs:
        input(f'\nМишень на {r} мм от передней грани корпуса (по рулетке). Enter — измерить ')
        vals = [d['raw'] for d in n.collect('U', 3.0, sd) if d['raw'] > 0]
        if len(vals) < 8:
            print('  мало ответов — мишень вне конуса или дальше «M»; повторите'); return
        m, dev = statistics.mean(vals), statistics.pstdev(vals)
        meas.append((r, m)); print(f'  среднее {m:.1f} мм, разброс {dev:.1f} мм, отсчётов {len(vals)}')
        if dev > 5: print('  разброс > 5 мм: проверьте крепление и отражения от соседних предметов')
    xs, ys = [m for r, m in meas], [r for r, m in meas]
    k = len(xs); mx, my = sum(xs) / k, sum(ys) / k
    sxx = sum((x - mx) ** 2 for x in xs)
    A = sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / sxx if sxx else 1.0
    B = my - A * mx
    err = max(abs(A * x + B - y) for x, y in zip(xs, ys))
    print(f'\nПоправка: истинное = {A:.4f} * измеренное + {B:.1f} мм; остаточная ошибка {err:.1f} мм')
    input('\nУберите мишень: перед тележкой свободный путь 3 м. Enter — 10 с проверки ')
    vals = [d['med'] for d in n.collect('U', 10.0, sd)]
    free = sum(1 for v in vals if v < 0) / max(1, len(vals))
    near = [v for v in vals if v > 0]
    print(f'  «пусто» в {free * 100:.0f} % отсчётов' + (f', ближайшее эхо {min(near)} мм' if near else ''))
    if free < 0.95:
        print('  ВНИМАНИЕ: датчик видит головку рельса, балласт или шпалы. Увеличьте наклон вверх (дуговой паз) или уменьшите M.')
    cfg_save({f'us_a_{sd}': round(A, 4), f'us_b_{sd}': round(B, 1), f'us_free_ratio_{sd}': round(free, 3)})
    n.send('N Q1')


def m_imu_calib(a):
    n = Node(a.port)
    input('Обе тележки стоят неподвижно на рельсах, привод выключен. Enter — калибровка смещений обоих IMU (2 с) ')
    n.send('N C')
    ok = 0
    for k, d in n.lines(7):
        if k == '#':
            print('  узел:', d['text'])
            ok += 'готово' in d['text']
            if ok == 2:
                break
    if ok < 2:
        print('Не оба IMU ответили «готово» — проверьте подключение (режим monitor, адреса 0x68 / 0x69)'); return
    input('Устройство на ровном горизонтальном участке (уровень на поперечине). Enter — 5 с замера ')
    rows = n.collect('I', 5.0)
    upd = {}
    for sd in ('A', 'B'):
        I = [d for d in rows if d['side'] == sd]
        if len(I) < 100:
            print(f'{sd}: мало отсчётов IMU'); continue
        rl = statistics.mean(roll_pitch(d['ax'], d['ay'], d['az'])[0] for d in I)
        pt = statistics.mean(roll_pitch(d['ax'], d['ay'], d['az'])[1] for d in I)
        pk = sorted(d['pk'] for d in I)
        noise = statistics.pstdev(d['az'] for d in I); p999 = pk[int(0.999 * (len(pk) - 1))]
        gn = max(statistics.pstdev(d[g] for d in I) for g in ('gx', 'gy', 'gz'))
        print(f'{sd}: крен установки {rl:+.2f}°, тангаж {pt:+.2f}°; шум az {noise:.1f} мг, пик |az-1g| (99,9 %) {p999} мг, шум гироскопа {gn:.2f} °/с')
        if abs(rl) > 5 or abs(pt) > 5:
            print(f'   ВНИМАНИЕ: перекос > 5° — проверьте, что корпус IMU {sd} сел на балку ровно')
        upd.update({f'imu_roll0_{sd}': round(rl, 2), f'imu_pitch0_{sd}': round(pt, 2), f'imu_noise_pk_mg_{sd}': p999})
    if upd:
        cfg_save(upd)


def peaks(rows, thr, gap_ms=300):
    """события: пики pk выше порога, не ближе gap_ms друг к другу -> [(t, pk)]"""
    ev = []
    for d in rows:
        if d['pk'] >= thr:
            if ev and d['t'] - ev[-1][0] < gap_ms:
                if d['pk'] > ev[-1][1]:
                    ev[-1] = (ev[-1][0], d['pk'])
            else:
                ev.append((d['t'], d['pk']))
    return ev


def m_imu_steps(a):
    n = Node(a.port); sd = a.side
    c = cfg_load()
    base = c.get(f'imu_noise_pk_mg_{sd}', 150)
    if not a.calib:
        thr = c.get(f'imu_thr_detect_mg_{sd}', base * 1.5)
        print(f'IMU {sd}: запись {a.seconds} с, порог события {thr:.0f} мг. Двигайте устройство на рабочей скорости.')
        rows = n.collect('I', a.seconds, sd)
        with open(a.out, 'w', encoding='utf-8') as f:
            f.write('t_ms,ax_mg,ay_mg,az_mg,gx_dps,gy_dps,gz_dps,pk_mg\n')
            for d in rows:
                f.write(','.join(str(d[k]) for k in ('t', 'ax', 'ay', 'az', 'gx', 'gy', 'gz', 'pk')) + '\n')
        crit = c.get(f'imu_thr_critical_mg_{sd}')
        for t, p in peaks(rows, thr):
            cls = 'КРИТИЧЕСКИЙ' if crit and p >= crit else 'ступенька'
            print(f'  t={t / 1000:.2f} с  пик {p} мг  -> {cls}')
        print(f'Журнал: {a.out}')
        return
    res = {}
    for h in (2, 4):
        P = []
        for k in range(a.passes):
            input(f'\nПластина {h} мм на головке рельса {sd} (стык-имитатор). Проезд {k + 1}/{a.passes}: Enter и сразу пускайте устройство через пластину ')
            rows = n.collect('I', a.seconds, sd)
            P.append(max(d['pk'] for d in rows) if rows else 0)
            print(f'  максимальный пик {P[-1]} мг')
        res[h] = statistics.median(P)
    p3, p7 = res[2], res[4]
    thr_det = max(base * 1.5, 0.6 * p3)
    thr_crit = (p3 + p7) / 2
    print(f'\nМедианы: 2 мм — {p3:.0f} мг, 4 мм — {p7:.0f} мг, шум — {base:.0f} мг')
    if p3 < base * 1.5:
        print('ВНИМАНИЕ: 2 мм почти не отличается от шума — увеличьте скорость проезда или проверьте жёсткость крепления IMU')
    if p7 <= p3 * 1.2:
        print('ВНИМАНИЕ: 4 мм мало отличается от 2 мм — величину ступеньки берите по дальномеру высоты (tof-live)')
    cfg_save({f'imu_pk_2mm_{sd}': p3, f'imu_pk_4mm_{sd}': p7, f'imu_thr_detect_mg_{sd}': round(thr_det), f'imu_thr_critical_mg_{sd}': round(thr_crit)})


# ------------------------------------------------------------------ дальномеры CJMCU-531 (VL53L1X)
# H — высотный: сырое расстояние до головки рельса (~53 мм; лицо корпуса Z 52). Выход tof_h: + = поверхность выше нуля.
# G — колейный: сырое расстояние до рабочей грани головки. Колея = tofG_off + kA * rawA + kB * rawB.
G_OFF_DEFAULT = 1398.2      # расстояние между чипами G A и G B по модели ред.7 при колее 1520 (уточняется калибровкой)
GAUGE_NOM = 1520.0


def tof_h(c, sd, raw):
    return c.get(f'tofH_a_{sd}', -1.0) * raw + c.get(f'tofH_b_{sd}', 53.0)


def tof_gauge(c, ra, rb):
    return c.get('tofG_off', G_OFF_DEFAULT) + c.get('tofG_k_A', 1.0) * ra + c.get('tofG_k_B', 1.0) * rb


def tof_collect(n, seconds, kind, side):
    """[raw] годных отсчётов (статус 0) и доля годных"""
    rows = [d for d in n.collect('T', seconds, side) if d['kind'] == kind]
    good = [d['raw'] for d in rows if d['st'] == 0]
    return good, (len(good) / len(rows) if rows else 0.0)


def m_tof_check(a):
    n = Node(a.port); n.send('N Q0')
    print('Тележки неподвижны на рельсах. 10 с замера всех дальномеров...')
    rows = n.collect('T', 10.0)
    n.send('N Q1')
    ok = True
    for sd in ('A', 'B'):
        for kind, nom, hz in (('H', 53, 30), ('G', 61, 10)):
            r = [d for d in rows if d['side'] == sd and d['kind'] == kind]
            g = [d['raw'] for d in r if d['st'] == 0]
            name = f'{kind} {sd}'
            if len(r) < hz * 10 * 0.6:
                print(f'{name}: отсчётов {len(r)} из ~{hz * 10} — датчик молчит или сбоит (команда ? , провод, XSHUT)'); ok = False; continue
            share = len(g) / len(r)
            if not g:
                print(f'{name}: нет годных отсчётов (статусы {sorted(set(d["st"] for d in r))}) — окно закрыто, пятно мимо головки'); ok = False; continue
            m, sdv = statistics.median(g), statistics.pstdev(g)
            print(f'{name}: медиана {m:.0f} мм (ожидается ~{nom}), шум {sdv:.2f} мм, годных {share * 100:.0f} %')
            if share < 0.9:
                print('   ВНИМАНИЕ: годных < 90 % — протрите окно, проверьте, что пятно на головке, а не на кромке'); ok = False
            if not (40 <= m <= 80):
                print('   ВНИМАНИЕ: расстояние вне 40..80 мм — кронштейн собран не так или датчик смотрит мимо головки'); ok = False
            if sdv > 3:
                print('   ВНИМАНИЕ: шум > 3 мм — засветка солнцем/блеск; проверьте бленду (козырёк) и крепление'); ok = False
    print('Итог:', 'норма' if ok else 'есть замечания (см. выше)')


def m_tof_height_calib(a):
    n = Node(a.port); sd = a.side
    n.send('N Q0')
    plates = [float(x) for x in a.plates.split(',')]
    print(f'Дальномер высоты тележки {sd}. Тележка неподвижна на ЦЕЛОМ рельсе (не у стыка), привод выключен.')
    print('Пластины — щупы / калиброванные шайбы известной толщины, ровные, матовые (заклейте блестящие малярным скотчем,'
          ' тогда и 0 мм меряйте на скотче). Кладите пластину на головку точно под окно дальномера.')
    meas = []; noise0 = 0.7
    for h in plates:
        input(f'\n{"Чистая головка рельса" if h == 0 else f"Пластина {h:g} мм под окном"}. Enter — 3 с замера ')
        g, share = tof_collect(n, 3.0, 'H', sd)
        if len(g) < 30:
            print(f'  мало годных отсчётов ({len(g)}, годных {share * 100:.0f} %) — проверьте пятно и окно; повторите'); n.send('N Q1'); return
        m, dev = statistics.median(g), statistics.pstdev(g)
        meas.append((h, m)); print(f'  медиана {m:.1f} мм, шум {dev:.2f} мм, отсчётов {len(g)}')
        if h == 0:
            noise0 = max(dev, 0.3)
    xs, ys = [m for h, m in meas], [h for h, m in meas]
    k = len(xs); mx, my = sum(xs) / k, sum(ys) / k
    sxx = sum((x - mx) ** 2 for x in xs)
    A = sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / sxx if sxx else -1.0
    B = my - A * mx
    err = max(abs(A * x + B - y) for x, y in zip(xs, ys))
    print(f'\nВысота над нулём = {A:.4f} * сырое + {B:.2f} мм; остаточная ошибка {err:.2f} мм (масштаб ожидается около -1)')
    if not (-1.25 <= A <= -0.8):
        print('ВНИМАНИЕ: масштаб далеко от -1 — пластины не под пятном или пятно задевает кромку головки; повторите')
    if err > 0.7:
        print('ВНИМАНИЕ: ошибка > 0,7 мм — порог 1 мм различим ненадёжно: увеличьте время накопления или проверьте крепление')
    print(f'Шум одного отсчёта {noise0:.2f} мм -> наименьшая надёжно обнаруживаемая ступенька {3 * noise0 * math.sqrt(2 / 6):.1f} мм')
    cfg_save({f'tofH_a_{sd}': round(A, 4), f'tofH_b_{sd}': round(B, 2), f'tofH_err_{sd}': round(err, 2), f'tofH_noise_{sd}': round(noise0, 2)})
    n.send('N Q1')


def m_tof_gauge_calib(a):
    n = Node(a.port); n.send('N Q0')
    c = cfg_load()
    upd = {}
    print('Дальномеры колеи A и B смотрят на рабочие грани своих рельсов в ОДНОМ сечении пути (X 120 обеих тележек).')
    if a.block > 0:
        for sd in ('A', 'B'):
            input(f'\n{sd}: тележка неподвижна. Enter — замер без пластины ')
            g0, _ = tof_collect(n, 3.0, 'G', sd)
            input(f'{sd}: прижмите пластину {a.block:g} мм к рабочей грани головки под окном дальномера. Enter — замер ')
            g1, _ = tof_collect(n, 3.0, 'G', sd)
            if len(g0) < 10 or len(g1) < 10:
                print('  мало годных отсчётов — повторите'); n.send('N Q1'); return
            d = statistics.median(g0) - statistics.median(g1)
            k = a.block / d if d > 0 else 1.0
            print(f'  разница {d:.1f} мм -> масштаб {k:.3f}')
            if not (0.85 <= k <= 1.15):
                print('  ВНИМАНИЕ: масштаб далеко от 1 — пластина не в пятне; масштаб не сохранён'); k = 1.0
            upd[f'tofG_k_{sd}'] = round(k, 4); c[f'tofG_k_{sd}'] = k
    s = input('\nИзмерьте ширину колеи путевым шаблоном ровно в сечении дальномеров (на 13 мм ниже верха головки).\n'
              'Введите ширину колеи, мм (например 1521): ').strip().replace(',', '.')
    try:
        true = float(s)
    except ValueError:
        print('не число'); n.send('N Q1'); return
    ga, _ = tof_collect(n, 5.0, 'G', 'A')
    gb, _ = tof_collect(n, 5.0, 'G', 'B')
    if len(ga) < 20 or len(gb) < 20:
        print('мало годных отсчётов A или B — команда tof-check'); n.send('N Q1'); return
    ra, rb = statistics.median(ga), statistics.median(gb)
    off = true - c.get('tofG_k_A', 1.0) * ra - c.get('tofG_k_B', 1.0) * rb
    print(f'  A {ra:.1f} мм, B {rb:.1f} мм -> поправка {off:.1f} мм (по модели около {G_OFF_DEFAULT})')
    if abs(off - G_OFF_DEFAULT) > 25:
        print('  ВНИМАНИЕ: поправка отличается от модели > 25 мм — проверьте упоры поперечин P29 и кронштейны P39/P40')
    upd['tofG_off'] = round(off, 1)
    cfg_save(upd)
    n.send('N Q1')


def m_tof_live(a):
    n = Node(a.port)
    c = cfg_load()
    thr = sorted(float(x) for x in a.thr.split(','))
    W = 6                                   # окно: 6 отсчётов H (~0,2 с) до и после
    floor = {sd: 3 * c.get(f'tofH_noise_{sd}', 0.7) * math.sqrt(2 / W) for sd in ('A', 'B')}   # 3 сигмы разности средних
    hist = {'A': [], 'B': []}; last_ev = {'A': 0, 'B': 0}; pend = {'A': None, 'B': None}
    ga = gb = None; t_print = 0
    print(f'Ctrl+C — выход. Ступенька: среднее {W} отсчётов после минус {W} до; пороги {thr} мм; '
          f'порог шума A {floor["A"]:.1f} / B {floor["B"]:.1f} мм. Журнал: {a.out}')
    print('Скорость при измерении — не больше 1 м/с (иначе окно длиннее участка между колёсами).')
    f = open(a.out, 'w', encoding='utf-8'); f.write('t_ms,side,kind,raw_mm,status,value_mm\n')
    try:
        for k, d in n.lines():
            if k == '#':
                print('\nузел:', d['text']); continue
            if k != 'T':
                continue
            sd = d['side']
            val = ''
            if d['st'] == 0:
                if d['kind'] == 'H':
                    h = tof_h(c, sd, d['raw']); val = f'{h:.2f}'
                    H = hist[sd]; H.append((d['t'], h)); del H[:-2 * W - 1]
                    if len(H) == 2 * W + 1:
                        before = statistics.mean(v for _, v in H[:W]); after = statistics.mean(v for _, v in H[W + 1:])
                        st = after - before
                        cand = pend[sd]
                        if abs(st) >= max(thr[0], floor[sd]) and d['t'] - last_ev[sd] > 600:
                            if cand is None or abs(st) > abs(cand[1]):
                                pend[sd] = (H[W][0], st)          # растёт — ждём максимума (окна полностью по разные стороны стыка)
                        cand = pend[sd]
                        if cand is not None and (abs(st) < 0.8 * abs(cand[1]) or d['t'] - cand[0] > 1000):
                            t_ev, stv = cand; pend[sd] = None; last_ev[sd] = d['t']
                            lvl = [x for x in thr if abs(stv) >= x]
                            print(f'\n*** {sd}: ступенька {stv:+.1f} мм (t={t_ev / 1000:.2f} с) — превышен порог {lvl[-1]:g} мм'
                                  + ('  [ВЫСШИЙ ПОРОГ]' if lvl[-1] == thr[-1] else ''))
                            f.write(f'{t_ev},{sd},STEP,,,{stv:.2f}\n')
                else:
                    if sd == 'A': ga = d['raw']
                    else: gb = d['raw']
            f.write(f'{d["t"]},{sd},{d["kind"]},{d["raw"]},{d["st"]},{val}\n')
            if time.time() - t_print > 0.3:
                t_print = time.time()
                hs = '  '.join(f'H {s_}: {hist[s_][-1][1]:+5.1f}' if hist[s_] else f'H {s_}:  нет' for s_ in ('A', 'B'))
                if ga is not None and gb is not None:
                    gg = tof_gauge(c, ga, gb); dg = gg - GAUGE_NOM
                    note = ' ЗАКРЫТИЕ (<1512 / >1548)' if gg < 1512 or gg > 1548 else (' вне допуска -4/+8' if dg < -4 or dg > 8 else '')
                    gs = f'колея {gg:7.1f} ({dg:+.1f}){note}'
                else:
                    gs = 'колея нет данных'
                sys.stdout.write('\r' + hs + '  |  ' + gs + '        '); sys.stdout.flush()
    except KeyboardInterrupt:
        print()
    finally:
        f.close()

def main():
    p = argparse.ArgumentParser(description='Датчики тележки: проверка и калибровка')
    sp = p.add_subparsers(dest='mode', required=True)
    sp.add_parser('ports')
    for name in ('monitor', 'us-calib', 'imu-calib', 'imu-steps', 'tof-check', 'tof-height-calib', 'tof-gauge-calib', 'tof-live', 'cmd'):
        q = sp.add_parser(name)
        q.add_argument('--port', default='/dev/ttyUSB0')
        if name == 'cmd':
            q.add_argument('text')
        if name == 'us-calib':
            q.add_argument('--refs', default='300,500,800,1200', help='эталонные расстояния, мм')
        if name == 'imu-steps':
            q.add_argument('--calib', action='store_true')
            q.add_argument('--passes', type=int, default=3)
            q.add_argument('--seconds', type=float, default=6.0)
            q.add_argument('--out', default='imu_steps.csv')
        if name in ('us-calib', 'imu-steps', 'tof-height-calib'):
            q.add_argument('--side', choices=('A', 'B'), required=True, help='тележка: A — левая с роликами, B — правая')
        if name == 'tof-height-calib':
            q.add_argument('--plates', default='0,1,2,4,5', help='толщины пластин (щупов), мм; первая — 0 (чистый рельс)')
        if name == 'tof-gauge-calib':
            q.add_argument('--block', type=float, default=0.0, help='толщина плоской пластины для проверки масштаба, мм (0 — не проверять)')
        if name == 'tof-live':
            q.add_argument('--thr', default='1,2,4,5', help='пороги ступеньки, мм (табл. 3.1 распоряжения 2288р)')
            q.add_argument('--out', default='tof_log.csv')
    a = p.parse_args()
    {'ports': m_ports, 'monitor': m_monitor, 'cmd': m_cmd, 'us-calib': m_us_calib, 'imu-calib': m_imu_calib,
     'imu-steps': m_imu_steps, 'tof-check': m_tof_check, 'tof-height-calib': m_tof_height_calib,
     'tof-gauge-calib': m_tof_gauge_calib, 'tof-live': m_tof_live}[a.mode](a)


if __name__ == '__main__':
    main()
