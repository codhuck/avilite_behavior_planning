# Development history

This archive preserves earlier requirements, local demo instructions and validation
notes. Profile names, machine paths and ground-truth descriptions refer to those
experiments; they are not the current installation guide. Start with the
[README](../README.md) for the supported setup workflow.


The plugin uses OpenDRIVE regulatory elements to choose driving behavior in
`LocalPlanningPipeline`. The map-aware dispatcher selects a global planner and
a local pipeline; it does not switch perception stacks.

## Original dispatcher work

### Scope checked against the supplied conversation

The 14:21 recording `11-03-2026 15.35.m4a` was transcribed locally on 9 November.
The machine transcript is in `logs/dispatcher/audio_requirements_transcript.json`;
timestamps below locate the source discussion, not a verbatim certified transcript.

| Recording | Requested development direction | Implementation / evidence |
|---|---|---|
| 00:15–01:27 | Behavior planning and execution-level replacement of planners | `PlannerDispatcher`, `PlannerSwitchTask`, map-selected global/local algorithms |
| 00:27–01:16 | STOP, zebra crossings, red/green traffic lights | `HDMapBehaviorPlanner`, `HDMapRuleVelocityPlanner`; CARLA scenario results in `VALIDATION.md` |
| 08:18–09:07 | Inspect OpenDRIVE HD-map elements and create behavior logic | `regulatory.py` extracts signals, lane applicability and crossing polygons |
| 09:15–09:49 | A behavior plugin in LocalPlanningPipeline; reusable profile | Automatic strategy registration, road/stop/junction profiles |
| 09:49–12:23 | Install/use CARLA bridge and package a working profile | Installed bridge and `dispatcher-road`; previously recorded physical CARLA validation |

The recording describes perception swapping as an example of execution-task
capabilities, not a specified perception algorithm to implement. Mapping,
keyboard/joystick controls, editor shortcuts and logs are a tour of existing
features. A future real-car trial is explicitly after simulation testing; no
real-car validation is claimed here. Race selection and Yas Marina were later
requested in the chat and are implemented separately in BasicSim.

### Execution-level replacement

Both dispatcher profiles include `PlannerSwitchTask`. With the vehicle stopped,
click **Select from map** in Planning: an execution event replaces the complete
global/local planning pair for the current map and route. The status line shows
the actual behavior, path and velocity stages and the current decision/reason.
This does not restart the simulator. During driving, the race path stage still
switches between reference following, checked lattice passing and corridor holding.

Other execution tasks can request replacement by notifying
`SelectPlanningStack(global_plan, expected_map)` through `executer.task_runner`.
The candidate route is copied, a first local plan is computed and stack
requirements are validated before either planner is published. Successful
replacement resets controller integrators and clears the held control command.
Unknown maps, stale requests, distant routes, moving vehicles and failed
validation leave the existing planner pointers intact. The task supports
`SyncExecuter`; threaded execution requires a separate synchronization protocol.
It does not load a new world or teleport between CARLA and BasicSim.

`tools/check_planner_switch.py` starts with a plain ReferencePathPlanner,
requests the full dispatcher stack via the execution task, then drives for
20 simulated seconds. It verifies that a moving switch is refused without
replacement, brakes the vehicle to rest, replaces again and drives for another
10 seconds. The whole sequence is repeated three times on Yas Marina.

Select installed profile `dispatcher-road` for Town10HD/CARLA or
`dispatcher-race` for Yas Marina/BasicSim. Both use `GlobalPlannerDispatcher`
and `PlannerDispatcher`.

| Map | Global planner | Behavior / path / velocity |
|---|---|---|
| HDMap | HDMapLaneFollowingPlanner | HDMapBehaviorPlanner / ReferencePathPlanner / HDMapRuleVelocityPlanner |
| RaceMap | GlobalCenterlineRacePlanner | RaceBehaviorPlanner / RacePathPlanner / RaceVelocityPlanner |

Selection uses the concrete map type, never the map filename or a missing
signal. Unknown map types raise an error. Switching is performed on replacement
of the global route and discards previous local plans and stop state. Changing
the map without replacing the route raises an error rather than continuing on
the old trajectory. The race pipeline now selects reference following, a checked lattice passing
path, and a passing corridor. Transitions between road and track within one
mixed map are not supported.

Yas Marina assets are `avlite/data/race_boundary_yas_marina.map.json` and
`yas_marina_real_race_line_mue_0_5_3_m_margin.json`. The new race profile uses
a generated centerline capped at 5 m/s for initial testing. Clicking global
replan uses the dispatcher's 10 m/s race cap. The installed CARLA map list does
not include Yas Marina: this profile uses BasicSim, not a CARLA race world.
`tools/prepare_dispatcher_profiles.py` generates reviewable profiles and the
test trajectory. `tools/check_dispatcher_race.py` checks the installed race
profile for 20 seconds; it is not a complete-lap validation.

- `HDMapLaneFollowingPlanner`: builds connected lane-following routes, excluding lateral graph edges until a lane-change path planner is available.
- `HDMapBehaviorPlanner`: Stop approach, continuous stopped dwell, traffic-light
  waiting, occupied-crosswalk yielding and reduced speed at clear crossings.
- `HDMapStopBehaviorPlanner`: compatibility alias with the same behavior.
- `HDMapRuleVelocityPlanner`: obstacle speed profile plus a stopping envelope;
  copies the trajectory so a stop does not permanently zero future speeds. It also caps speed by curvature and propagates braking upstream.
- `CarlaRegulatoryTask`: reads simulator ground truth after each executor tick,
  supplies simulation time, and writes CSV motion/decisions and JSONL events.
  It does not advance CARLA or change traffic lights.

## Installed profiles

Choose `hdmap-stop-west`, `hdmap-stop-east`, or `hdmap-junctions` in AVLite's
profile selector, then reload the stack. The existing `avlite-bridge-carla`
profile now uses the west scenario too. Original settings were backed up next
to the installed YAML files. Run only one AVLite instance against the server.

```yaml
c20_planning:
  c23_behavioral_strategy: HDMapBehaviorPlanner
  c23_path_strategy: ReferencePathPlanner
  c23_velocity_strategy: HDMapRuleVelocityPlanner
c40_execution:
  c40_global_planner: HDMapLaneFollowingPlanner
  c40_local_planner: LocalPlanningPipeline
  c40_execution_tasks: [CarlaRegulatoryTask]
  c40_log_level: INFO
  c40_log_to_file: true
```

Profiles use no randomly spawned background cars and a 3 m/s route speed.
Ordinary GUI runs observe current CARLA lights/pedestrians; they do not arrange
controlled red lights or spawn test pedestrians. The automated runner does.
Telemetry is in `logs/hdmap/drive_*.csv` and `.jsonl`. Main application logs are
in `logs/`. CSV includes simulation time, actual/target speed, route position,
lateral error, behavior state, control ID/distance and dynamic observations.
The `phase` column distinguishes `APPROACHING` a stop target from `WAITING`
at it; `wait_duration` measures continuous waiting in simulation seconds and
resets on release, movement, or a change of controlling rule/reason. A
`WAIT_LIGHT` state alone does not mean the vehicle is stationary.
`ARRIVED / route_complete` indicates the vehicle has stopped within 0.8 m of
the route endpoint (also checked along the route), with no active regulatory
stop. This is a diagnostic state; it does not turn off the simulation loop.

Crosswalk clearance uses the actual polygon's intersection with the route.
The crossing remains active until the rear bumper passes that intersection's
exit; it is not discarded at a fixed distance from its entrance. Pedestrian
occupancy still uses CARLA ground truth and does not predict pedestrian intent.

For stationary vehicle obstacles the velocity stage caps the reference speed
by available braking distance. It does not seed the profile with measured ego
speed: doing that trapped the planner at near-zero speed after a stop.
Close-obstacle stops and regulatory stop envelopes still apply. Moving-lead
following retains the base planner's behavior.

Run `PYTHONNOUSERSITE=1 python tools/check_hdmap_departure.py` with AVLite closed
and an empty Town10HD_Opt scene to check departure and stopping before a parked
vehicle. Results and sampled motion are saved to
`logs/hdmap_validation_departure_fix/parked_vehicle.json`.

## Repeatable validation

Close AVLite and other clients that advance the world. Start CARLA on port 2000
with Town10HD_Opt, then run from the repository root:

```bash
conda activate avlite-carla
PYTHONNOUSERSITE=1 python tools/run_hdmap_scenarios.py --repeats 2 --output logs/hdmap_validation_new
```

The runner uses real vehicle physics, the behavior pipeline and Stanley control.
For a physically walking pedestrian, use
`--routes junctions --walking-pedestrian --repeats 2`. The two repeats walk in
opposite directions, then along the sidewalk. No pedestrian is teleported or
removed to clear the crossing. A run fails unless the walker starts, leaves
the occupied area, and the vehicle completes the route. Sampled pedestrian
positions are saved in `pedestrian_motion.json`. This is a scripted crossing,
not autonomous pedestrian navigation or intent prediction.
It checks front-bumper crossings, collisions, arrival, lateral error (at most 1 m), and continuous Stop dwell independently from recorded speed and position.
It uses direct simulator observations rather than the bridge's RGB/LiDAR path.
`tools/smoke_hdmap_stack.py` separately checks that installed profiles, bridge,
sensors, executor and recording work together (short run, not a route verdict).
`tools/prepare_hdmap_profiles.py` rebuilds reviewable route/profile files from
an existing installed CARLA profile without installing them.

Self-intersecting routes are rejected explicitly because repeated passages need progress-aware map matching. Lane-change-required goals may have no route or require a longer detour.

Stop timing uses simulation seconds. Lane validity and signal references select
applicable controls; saved routes retain lane identifiers. Unknown or stale
light/pedestrian observations cause conservative waiting. A missed Stop is
reported as a violation rather than silently marked complete.

See [VALIDATION.md](../VALIDATION.md) for measured results and limitations.

## Remaining development scope

The requested direction is HD-map-aware behavior in AVLite/CARLA, not joystick
control. The dispatcher selects road/race planners by map type. Race passing is
experimental; the extended suite includes a tight corner, two rivals, stop/go,
lap seam and complete laps, but does not cover arbitrary opponent behavior.
Real camera-based recognition, pedestrian intent
prediction, lane changes and arbitrary traffic remain unimplemented or unvalidated.
Low-throttle departure delay in the CARLA bridge is separate from the fixed
near-zero-speed deadlock in stationary-obstacle planning.

## Experimental race behavior

For a visible demonstration choose `dispatcher-race-traffic` and press Start.
`RaceTrafficTask` adds two scripted cars at 18 and 43 m ahead, moving at 2 m/s
along Yas Marina in BasicSim. Reset removes its own cars before recreating them;
it does not add traffic to CARLA. This reproduces the slower-rival demonstration
in the actual GUI instead of an empty track. Keep execution Tasks enabled.

`RaceBehaviorPlanner` selects `RACE`, `FOLLOW`, and requests passing of a slower
leader. `RacePathPlanner` only accepts `OVERTAKE` after checking the lattice
proposal for predicted collisions and track clearance including body orientation.
If no acceptable proposal is available it follows the reference with a speed cap.
Once alongside it can hold a checked lateral corridor. The rival ID is retained
until rear-bumper clearance; `RETURN` then uses a checked lattice proposal to
rejoin the reference. A pass is deferred if the next 80 m contain a sharp
heading change (over 0.03 radians per metre). A failed return holds a checked
offset corridor or requests braking; it does not cut back onto an unchecked
reference. Ordinary tracking error does not activate the post-overtake return
state. Closed reference paths wrap at the lap seam.

Run the expanded suite with:

```bash
PYTHONNOUSERSITE=1 python tools/run_race_scenarios.py --repeats 2 --output logs/race_expanded_new
PYTHONNOUSERSITE=1 python tools/run_hdmap_scenarios.py --routes junctions stop_west --repeats 2 --walking-pedestrian --pedestrians 3 --dispatcher --max-sim-time 450 --output logs/hdmap_group_new
```

The race suite uses BasicSim dynamics and the real planner/controller with
scripted ground-truth rivals; it skips rendering sensors. It checks body
intersection, oriented body clearance and unwrapped route progress on every
0.05 s step. The CARLA suite must run with AVLite closed; its three walkers
start with staggered delays, walk out of the crossing and keep moving along
the sidewalk so they do not physically block the remaining walkers.

`RaceVelocityPlanner` applies behavior limits and the existing conservative
obstacle/curvature envelope. This is not a high-speed racing controller. State,
reason, rival and selected path changes are logged as `RACE ...` at INFO level.

Reproduce the scripted BasicSim checks with:

```bash
PYTHONNOUSERSITE=1 python tools/check_race_behavior.py --output logs/dispatcher/race_overtake_final.json
PYTHONNOUSERSITE=1 python tools/check_race_behavior.py --narrow --output logs/dispatcher/race_follow_narrow.json
```

The leader follows the reference at 2 m/s, with an initial 18 m gap. The normal
case requires passing, returning and resuming race mode. The narrow case limits
the usable corridor to 3 m and requires refusal of overtaking. Both check sampled
vehicle-body overlap and lateral body clearance. These tests cover 80 seconds
near the start of Yas Marina, not a full lap or arbitrary opponents.

### Запись полного круга с двумя соперниками

```bash
PYTHONNOUSERSITE=1 MPLBACKEND=Agg MPLCONFIGDIR=/tmp/avlite-mpl \
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 \
python tools/check_race_traffic_profile.py --full-lap \
  --video logs/race_full_traffic/full_lap.mp4 \
  --output logs/race_full_traffic/result.json
```

Загружается установленный профиль `dispatcher-race-traffic`, включая
`RaceTrafficTask`, исполнитель, восприятие и фиксированные шаги времени.
Прогресс разворачивается через стык трассы независимо от счётчика кругов UI.
Каждые 0,05 с проверяются число соперников, пересечение кузовов и выход
ориентированного кузова за боковые границы. При нарушении проезд завершается
с ненулевым кодом, JSON сохраняет причину и телеметрию.

MP4 — запись визуализации фактических состояний BasicSim, а не захват рабочего
стола и не окно CARLA. Общая карта показывает все три машины; локальный вид —
их кузова. Цвета: голубой — наш автомобиль, оранжевый/фиолетовый — соперники.
1280×720, 10 кадров/с, ускорение 5×; GPU и CARLA для записи не требуются.
Соперники следуют заданной траектории со скоростью 2 м/с.

### Городской проезд с видео CARLA

При запущенном CARLA Town10HD_Opt и закрытых других управляющих клиентах:

```bash
PYTHONNOUSERSITE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 \
python tools/run_hdmap_scenarios.py --routes stop_west --repeats 1 \
  --walking-pedestrian --pedestrians 3 --dispatcher --video \
  --max-sim-time 450 --output logs/city_video_retry
```

Собственная камера 1280×720 позади автомобиля записывает кадры каждого шага
CARLA с номером кадра, состоянием поведения, временем и скоростью. MP4 имеет
ускорение 2×, битрейт ограничен 1,2 Мбит/с. Режим рендеринга включается только
для записи и исходные настройки мира восстанавливаются после проезда.
Сценарий удаляет только созданные им объекты. Это управляемая проверка STOP,
светофоров и группы пешеходов, без движущегося автомобильного трафика.

Запись требует минимум 120 МиБ свободного диска; при остатке менее 30 МиБ
она прекращается. Фрагментированный MP4 сохраняет уже записанную часть при
аварии. Ошибка завершения записи не должна препятствовать удалению объектов.

#### Высокое качество городского видео

Добавить `--video-quality high` к команде записи: камера 2560×1440, H.264
CRF 18, максимум 20 Мбит/с, постобработка включена, motion blur и хроматическая
аберрация отключены. Нужно минимум 700 МиБ свободного места.
Сам сервер должен запускаться с `-quality-level=Epic`: разрешение камеры
само по себе не меняет качество графики Unreal Engine.

На этом компьютере проверен отдельный контейнер `avlite-carla-quality-1080`
с Epic, служебным буфером 1280×720 и камерой **2560×1440**. Имя контейнера
не определяет разрешение записи. Исходный `avlite-carla-server` с Low сохранён.
Одновременно запускать только один сервер: оба используют порт 2000.
После записи остановить сервер, чтобы он не продолжал нагружать GPU.

## Sensor-driven dynamic observations

See [SENSOR_PERCEPTION.md](../SENSOR_PERCEPTION.md) for the separate `sensor-road` profile, camera/LiDAR detector, setup, tests and retained map/localization assumptions. Existing ground-truth profiles remain available for regression comparisons.


## Meeting context

The supplied six-minute WebM contains English automatic captions and no audio
stream. Around 00:00–00:45 the discussion proposes perception updating the
perception model; around 01:10–03:00 it discusses model provenance, weight choice
and decoupling; around 03:00–05:55 it requests README/dependency clarity and
Python installation. These are paraphrases of captions, not a verbatim transcript.

The actual current planner is a Python rule-based state machine. The conversation
mentions behavior trees/C++, but the source contains no BehaviorTree.CPP or
py_trees implementation. Adding a wrapper would introduce an unused dependency.
A future behavior-tree implementation is separate work, not an existing feature.

