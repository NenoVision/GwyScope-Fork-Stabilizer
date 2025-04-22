# GwyScope Fork – Stabilizer / Setpoint Watcher Extension

## Rozšíření HWServer části projektu GwyScope o Setpoint Watcher

Tato verze je forkem původního open-source projektu [GwyScope](https://sourceforge.net/projects/gwyscope/), vyvíjeného pod licencí [GNU GPLv3](https://www.gnu.org/licenses/gpl-3.0.html), jehož původními autory jsou:

- Petr Klapetek <pklapetek@cmi.cz>
- Miroslav Valtr <miraval@seznam.cz>
- David Nečas (Yeti) <yeti@gwyddion.net>

Tento repozitář zahrnuje pouze upravenou část **HWServer**. Původní GUI ani frontend GwyScope nejsou součástí tohoto forku.

---

## 🧠 Přehled úprav

V rámci projektu **TACOM – TM03000033** (https://starfos.tacr.cz/cs/projekty/TM03000033) jsme do HWServeru implementovali nový modul **Setpoint Watcher**, který umožňuje:

- sledování hodnoty měřeného parametru během měření,
- automatickou stabilizaci výstupního parametru na požadovaný setpoint,
- bezpečné dlouhodobé měření bez zásahu operátora.

### 🔧 API pro komunikaci s backendem

Byly přidány nové příkazy do souboru `communication.c`:

- `set_stabilizer`
- `run_stabilizer`
- `stop_stabilizer`

Přenos dat probíhá přes TCP a v interním GWY formátu.

### 📜 Lua Interface

Pro ovládání stabilizace z Lua skriptů byla přidána funkce do `luafunctions.c`:

```c
int gws_stabilize_setpoint(lua_State* L);
```

Tato funkce spustí stabilizaci sondy a čeká na její dokončení. Vrací 1, pokud došlo k úpravě parametru (tedy byla provedena stabilizace).

---

## 🔍 Popis funkce Setpoint Watcher

Setpoint Watcher monitoruje průběžně hodnotu určitého kanálu (např. výstupní frekvence), porovnává ji s cílovou hodnotou (setpoint) a v případě odchylky automaticky upravuje řídicí parametr. Cílem je:

- Zamezit degradaci měření vlivem posunu frekvence,
- Automatizovat korekci bez zásahu uživatele,
- Zajistit stabilitu u dlouhodobých měření.

Použité parametry:
- `control_output` – stabilizovaný výstupní kanál (např. frekvence generátoru),
- `control_range` – povolená odchylka od cílové hodnoty,
- `step` – krok pro úpravu řídicího parametru,
- `setpoint` – cílová hodnota,
- `tuning_param` – parametr, který se mění pro dosažení stabilizace,
- `oversampling` – počet bodů použitých pro vyhodnocení stabilizace.

---

## ✅ Validace

Funkce byla validována ve dvou dlouhodobých měřeních (20–21 hodin) v prostředí vakua:

- **Měření Akiyama** – setpoint držen kolem 0 Hz, úspěšně stabilizováno.
- **Měření MFM** – obdobný průběh, rovněž úspěšná stabilizace.

---

## 📄 Licence

Tento repozitář je poskytován pod stejnou licencí jako původní projekt GwyScope: **GNU GPLv3**.

---

## Extension of GwyScope (HWServer part) – Setpoint Watcher

This repository is a modified fork of the open-source [GwyScope](https://sourceforge.net/projects/gwyscope/) project licensed under [GNU GPLv3](https://www.gnu.org/licenses/gpl-3.0.html). Original authors:

- Petr Klapetek <pklapetek@cmi.cz>
- Miroslav Valtr <miraval@seznam.cz>
- David Nečas (Yeti) <yeti@gwyddion.net>

Only the **HWServer** part is included. GUI/frontend was not modified.

---

## 🧠 Summary of Modifications

As part of the **TACOM – TM03000033** project (https://starfos.tacr.cz/cs/projekty/TM03000033), we implemented a new module called **Setpoint Watcher** into the HWServer. It enables:

- Continuous monitoring of a selected parameter,
- Automatic correction to maintain the desired setpoint,
- Long-term measurement stability without operator interaction.

### 🔧 New Backend API Commands

In `communication.c`, new TCP-based commands were added:

- `set_stabilizer`
- `run_stabilizer`
- `stop_stabilizer`

### 📜 Lua Interface

Added to `luafunctions.c`:

```c
int gws_stabilize_setpoint(lua_State* L);
```

This function allows Lua scripts to run stabilization and wait for completion. Returns 1 if the parameter was adjusted.

---

## 🔍 Setpoint Watcher Description

This feature monitors a real-time measurement parameter and automatically adjusts the hardware to maintain a defined setpoint. It’s crucial for preventing signal drift and ensuring high-quality measurements in long acquisitions.

Main parameters:
- `control_output` – monitored output channel,
- `control_range` – allowed deviation from setpoint,
- `step` – increment/decrement applied to adjust,
- `setpoint` – target value,
- `tuning_param` – parameter that is tuned,
- `oversampling` – number of samples to evaluate stability.

---

## ✅ Validation

Validated in two long-term vacuum experiments:
- **Akiyama probe**: ~20h, final value around 0 Hz.
- **MFM mode**: ~21h, also stable around 0 Hz.

---

## 📄 License

Provided under the same license as the original GwyScope project: **GNU GPLv3**.
