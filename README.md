# Compass16-LBM

> **Lattice-Boltzmann-inspirierte Schwarmkoordination mit ganzzahliger Kompassdiskretisierung.**
> Roboter, die wie eine Flüssigkeit fließen — ohne Trigonometrie, ohne FPU, lauffähig auf 8-Bit-Mikrocontrollern.

[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)
[![Status: Research](https://img.shields.io/badge/Status-Independent_Research-f472b6.svg)](#)
[![Math](https://img.shields.io/badge/Math-Integer_only-34d399.svg)](#)
[![Year](https://img.shields.io/badge/Year-2026-fbbf24.svg)](#)

**Autor:** Dogan Balban · ORCID [0009-0002-5052-6951](https://orcid.org/0009-0002-5052-6951) · GitHub [@dogan1908](https://github.com/dogan1908)

---

## English Summary (TL;DR)

Compass16-LBM ports the Lattice-Boltzmann method to decentralized robot swarms. Each agent stores a 17-value distribution vector (16 compass directions + rest) and updates it via integer-only BGK relaxation. Out of these purely local rules emerge fluid-like global behaviors — collision-free flow around obstacles, lane formation in narrow corridors, deadlock-free goal seeking. Designed to run on 8-bit MCUs (ATmega328) with **38 bytes per agent** and **~276 cycles per timestep**. Live browser simulation included.

---

## Was ist das?

Die klassische **Lattice-Boltzmann-Methode (LBM)** simuliert Flüssigkeiten durch zwei lokale Operationen:

1. **Relaxation** — jeder Gitterknoten nähert sich einem lokalen Gleichgewicht an
2. **Streaming** — Verteilungswerte wandern zu Nachbarn

Aus diesen rein lokalen Regeln entsteht makroskopisch das Navier-Stokes-Verhalten — *obwohl kein Knoten die Gleichungen kennt oder löst*.

**Compass16-LBM überträgt dieses Prinzip auf mobile Agenten.** Statt Fluidteilchen koordinieren wir Roboter, die sich fließend und kollisionsfrei bewegen — ohne zentrale Steuerung, rein durch lokale Interaktion.

### Kernidee in einer Formel

```
f_k^eq = w_k · (ρ + κ · (c_k · u_target))
```

Jeder Agent zieht seinen 17-Werte-Verteilungsvektor `f` über BGK-Relaxation in Richtung dieses Gleichgewichts. `u_target` setzt sich zusammen aus Navigation (Ziel), Separation (Kollisionsvermeidung) und Kohäsion (Schwarm-Zusammenhalt) — Reynolds-Boids in Integer-Form.

---

## Die fünf emergenten Eigenschaften

Diese Verhaltensweisen entstehen *emergent* aus den lokalen Regeln — sie werden nicht explizit programmiert.

| # | Eigenschaft | Bedingung | Analogie |
|---|-------------|-----------|----------|
| 1 | **Fließverhalten um Hindernisse** | ω ≈ 128, β > α | Flüssigkeit umfließt einen Stein |
| 2 | **Automatischer Abstandsausgleich** | Separation + Kohäsion | Moleküle in einer Flüssigkeit |
| 3 | **Geordneter Gegenverkehr** | Bounce-Back in Engstellen | Spurbildung in Strömungen |
| 4 | **Deadlock-Freiheit** | u_Ziel ≠ 0 ⇒ ρ-Fluss permanent | keine Patt-Situationen |
| 5 | **Viskosität ⇄ Trägheit** | ν ∝ (S/ω − ½) | Honig vs. Wasser per Slider |

Optional als sechste Schicht: **🐜 Pheromonkanal** (ACO-inspiriert) — über die Zeit entstehen Trampelpfade ohne explizite Pfadplanung.

---

## LBM-Begriffe ↔ Schwarm-Bedeutung

Die LBM-Literatur verwendet Begriffe aus der Fluiddynamik. Im Schwarmmodell haben diese Größen eine andere Bedeutung:

| LBM-Original | Schwarm-Bedeutung | Symbol |
|---|---|---|
| Gitterknoten | Agentenposition | x_A |
| Population f_i | Bewegungstendenz in Richtung i | f_k |
| Lokale Dichte ρ | Aktivitätslevel des Agenten | ρ |
| Makro-Geschwindigkeit u | Mittlere Bewegungsrichtung | u |
| Schallgeschwindigkeit c²_s | *entfällt* — ersetzt durch Reaktivität | κ |
| Druck | Interaktionsstärke (Nachbarabstoßung) | — |
| Viskosität | Trägheit der Richtungsänderung | ∝ (S/ω − ½) |
| Bounce-Back an Wänden | Ausweichen an Hindernissen | f_k ↔ f_k̄ |

---

## Compass16-Geschwindigkeitssatz

16 Richtungen im Abstand von 22,5° plus Ruhe — alle Komponenten ganzzahlig, vorberechnet, ROM-resident:

```
c_k = (⌊256·sin((k−1)π/8)⌋, ⌊256·cos((k−1)π/8)⌋)   für k = 1…16
c_0 = (0, 0)                                         (Ruhe)
```

Konkret:

| k | Richtung | c_x | c_y |   | k | Richtung | c_x | c_y |
|---|----------|-----|-----|---|---|----------|-----|-----|
| 0 | Ruhe | 0 | 0 |   | 9 | S | 0 | −256 |
| 1 | N | 0 | 256 |   | 10 | SSW | −98 | −237 |
| 2 | NNE | 98 | 237 |   | 11 | SW | −181 | −181 |
| 3 | NE | 181 | 181 |   | 12 | WSW | −237 | −98 |
| 4 | ENE | 237 | 98 |   | 13 | W | −256 | 0 |
| 5 | E | 256 | 0 |   | 14 | WNW | −237 | 98 |
| 6 | ESE | 237 | −98 |   | 15 | NW | −181 | 181 |
| 7 | SE | 181 | −181 |   | 16 | NNW | −98 | 237 |
| 8 | SSE | 98 | −237 |   |   |   |   |   |

Die **Antiparallelität** k̄ = ((k−1+8) mod 16) + 1 ist O(1) und macht Bounce-Back zu einem einzigen Vertausch.

---

## Live-Simulation

📂 **[`simulation.html`](simulation.html)** — Standalone-HTML, keine Build-Tools.

🌐 **[`index.html`](index.html)** — Projektseite mit Erklärungen.

### Features der Simulation

- 12 Roboter mit vollständiger LBM-Pipeline (Wahrnehmung → Sollbewegung → Gleichgewicht → Relaxation → Bounce-Back → Positionsupdate)
- 5 Stationstypen: 🔧 Werkzeug · 📦 Lager · ⚡ Aufladen · ⚙️ Montage · 📤 Abgabe
- 5 Szenarien: Wand · Offen · Labyrinth 1–3
- Frei zeichenbare Wände + Stationen + Radierer
- Optionale Pheromonschicht (ACO-inspiriert)
- Wall-Follow / Bug-Algorithmus für hartnäckige Sackgassen
- Mausagent (Roboter ✦) — manuell steuerbar
- Live-Statistiken: FPS, Step-Time, Punkte, Auftragslog

### Tastaturkürzel

| Taste | Aktion |
|---|---|
| `Space` | Pause / Play |
| `R` | Szenario zurücksetzen |
| `1`–`5` | Szenarien wechseln |
| `D` | Wände zeichnen |
| `S` | Station setzen |
| `E` | Radierer |
| `C` | Gezeichnete Wände löschen |
| `?` / `H` | Hilfe-Overlay |

### LBM-Parameter (Slider)

| Symbol | Bedeutung | Bereich | Standard |
|---|---|---|---|
| **κ** | Reaktivität — wie schnell wird *u_target* gefolgt | 1–32 | 4 |
| **ω** | Relaxation — Trägheit der Richtungsänderung | 16–256 | 128 |
| **α** | Navigation — Stärke der Zielanziehung | 0–512 | 144 |
| **β** | Separation — Kollisionsvermeidung | 0–512 | 160 |
| **γ** | Kohäsion — Schwarm-Zusammenhalt | 0–512 | 96 |
| **r** | Roboterradius | 12–35 px | 16 |
| **v** | Maximalgeschwindigkeit | 1–10 | 4 |

Alle LBM-Parameter sind als **Zweierpotenzen** wählbar — Multiplikationen werden zu Shifts.

---

## Hardware-Eckdaten

| Aspekt | Wert |
|---|---|
| Speicher pro Agent | **38 Bytes** (17×16-Bit f + 2×16-Bit Position) |
| Rechenkosten / Agent / Schritt | **~276 Zyklen** |
| Floating-Point-Ops | **0** |
| Skalierung | S = 256 = 2⁸ |
| Zielhardware | 8/16-Bit-MCUs (ATmega328 & Co.) |

Hochrechnung für ATmega328 @ 16 MHz:

| N (Agenten) | Zeit/Schritt | Update-Rate |
|---|---|---|
| 10 | 0,17 ms | >1000 Hz |
| 100 | 1,7 ms | ~580 Hz |
| 500 | 8,6 ms | ~116 Hz |
| 1000 | 17,3 ms | ~58 Hz |

Zum Vergleich: Reynolds-Flocking mit `√` und `arctan` benötigt ~2000–3000 Zyklen/Agent — **8–10× langsamer**.

---

## Repository-Struktur

```
.
├── index.html          → GitHub-Pages-Landingpage
├── simulation.html     → Live-Simulation (standalone)
├── manuscript.tex      → Mathematisches Arbeitsdokument v2
├── README.md           → Diese Datei
└── LICENSE             → MIT-Lizenz
```

---

## Lokal verwenden

Keine Installation nötig — alle HTML-Dateien sind standalone:

```bash
git clone https://github.com/dogan1908/compass16-lbm.git
cd compass16-lbm
# Datei direkt im Browser öffnen, oder lokal servieren:
python3 -m http.server 8000
# → http://localhost:8000
```

Das LaTeX-Dokument lässt sich mit jedem TeX-Distribution kompilieren:

```bash
pdflatex manuscript.tex
```

---

## GitHub Pages

Diese Seite ist GitHub-Pages-fähig. Im Repository-Settings unter *Pages*:

- **Source:** Deploy from a branch
- **Branch:** `main` / `(root)`

Die Seite wird unter `https://<username>.github.io/<repo>/` veröffentlicht.

---

## Verwandte Arbeit

Der DSM-Klassifikator (Compass16-Schwellwerte τ₁ = 51/256 und τ₂ = 171/256) stammt aus:

> Balban, D. (2026). *Trigonometry-Free TSP Heuristics for Embedded Systems: A Compass-Based Matching Approach.* Working Paper. DOI: [10.5281/zenodo.19096781](https://doi.org/10.5281/zenodo.19096781)

Compass16-LBM nutzt diesen Klassifikator als Wahrnehmungsschicht.

---

## Offene Fragen

Aus dem Manuskript v2:

1. **Isotropie der Gewichte** — exakte Isotropie 5. Ordnung mit gleichen Gewichten w_r = 1/24 ist Approximation. Numerische Validierung steht aus.
2. **Integer-Division in der Separation** — Lookup-Table vs. stückweise lineare Approximation vs. linear-statt-invers.
3. **3D-Erweiterung** — Compass16 × 5 Elevationsklassen (162 B/Agent) oder Compass32 × 3 (194 B/Agent).
4. **Stabilitätsanalyse** — Chapman-Enskog-Entwicklung auf Schwarmmodell übertragbar?
5. **Validierung gegen Baselines** — ORCA, CBS-MAPF, klassisches Reynolds-Flocking.

---

## Befunde aus der Code-Analyse (v7 → v7.1)

Bei der Analyse vor diesem Release wurde ein **Bug in der `compass16(dx,dy)`-Funktion** entdeckt und korrigiert. Die ursprünglichen Lookup-Tabellen lauteten:

```js
// FEHLERHAFT — Index 3 dupliziert Index 2:
[1, 2, 3, 3, 5]   // NE-Quadrant
[9, 8, 7, 7, 5]   // SE-Quadrant
[1, 16, 15, 15, 13]   // NW-Quadrant
[9, 10, 11, 11, 13]   // SW-Quadrant
```

Konsequenz: Die vier "Zwischenrichtungen" **ENE (k=4), ESE (k=6), WSW (k=12), WNW (k=14)** wurden **nie ausgegeben**. Der Klassifikator deckte effektiv nur 12 der 16 Sektoren ab — Eingaben, die eigentlich in einer der vier Zwischenrichtungen liegen, wurden zur jeweiligen Diagonalen kollabiert (z.B. ENE → NE).

Das Sichtbarkeitsverhalten war subtil: Die Bewegungsformel summiert weiterhin korrekte Vektoren `c_k`, aber die Zielklassifikation und Hindernismarkierung verlor bis zu 22,5° Genauigkeit in den betroffenen Bereichen. Die Roboter erreichten Ziele dennoch, da die mehrfachen Sensorbeiträge das Defizit kompensierten.

**Fix in v7.1** (in dieser Distribution enthalten):

```js
[1, 2, 3, 4, 5]   // NE: N, NNE, NE, ENE, E
[9, 8, 7, 6, 5]   // SE: S, SSE, SE, ESE, E
[1, 16, 15, 14, 13]   // NW: N, NNW, NW, WNW, W
[9, 10, 11, 12, 13]   // SW: S, SSW, SW, WSW, W
```

Test (1°-Sweep über 360°): nun werden alle 16 Sektoren mit jeweils ~22,5°-Spanne erreicht. Bewegungsverhalten in Engstellen wirkt nach dem Fix sichtbar glatter.

### Weitere Verbesserungen in v7.1

- Tastatur-Shortcuts (Space, R, 1–5, D, S, E, C, ?, H)
- FPS- und Step-Time-Counter (oben links)
- Hilfe-Overlay mit kompletter Tastatur- und Parameter-Referenz
- Persistenter Autoren-Credit unten links

---

## Zitation

```bibtex
@unpublished{balban2026compass16lbm,
  author       = {Balban, Dogan},
  title        = {{Compass16-LBM: Lattice-Boltzmann-inspirierte Schwarm-
                  koordination mit ganzzahliger Kompassdiskretisierung}},
  year         = {2026},
  month        = mar,
  note         = {Working Paper v2. Independent Research.},
  url          = {https://github.com/dogan1908/compass16-lbm}
}
```

---

## Lizenz

Code und Dokumentation: **[MIT-Lizenz](LICENSE)**.

Der Manuskripttext ist frei zur akademischen Verwendung mit Zitation.

---

<sub>© 2026 Dogan Balban · Independent Research · Erstellt mit Sorgfalt und ohne Floating-Point-Operationen.</sub>
