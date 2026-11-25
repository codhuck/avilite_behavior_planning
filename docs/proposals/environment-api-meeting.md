# Meeting notes: environment-observation API

Draft for Iaroslav. Not sent. The design is described in
[the proposal](environment-api.md). Speak about the main planning-plugin branch;
this update does not concern the neural selector.

## Short version, English (about one minute)

Following your feedback, I prepared a proposal for the perception interface.
I suggest putting a small environment-observation model in the core, with
shared types for traffic signals and crossing occupancy. Detection and driving
logic would stay in the plugins.

Each observation includes capture time, a coordinate frame and its source.
We also separate sensor evidence from simulator truth, and object identity
from its association with the map.
Unknown or stale data would stay unknown; it would not be treated as a green
light or a clear crossing.

The planner would declare which observations it needs, and the selected
perception module would declare what it can provide.

The local version is implemented and tested in the main branch. Moving the
shared types into the core is still a proposal. Does this separation make sense
before I prepare the core changes?

## Русский перевод

После ваших замечаний я подготовил предложение по интерфейсу восприятия.
Предлагаю добавить в ядро небольшую модель наблюдений окружения с общими типами
для сигналов светофоров и занятости переходов. Распознавание и логику движения
оставить в плагинах.

В каждом наблюдении есть время получения данных, система координат и источник.
Мы также отделяем сенсорные данные от данных симулятора, а идентификатор объекта —
от его привязки к карте. Неизвестные или
устаревшие данные останутся неизвестными: мы не будем считать их зелёным сигналом
или свободным переходом.

Планировщик будет объявлять, какие наблюдения ему нужны, а выбранный модуль
восприятия — какие данные он может предоставлять.

Локальная версия реализована и проверена в основной ветке. Перенос общих типов
в ядро пока остаётся предложением. Подходит ли такое разделение, прежде чем
я подготовлю изменения для ядра?

## If asked for details

**Why shared road types as well as a base class?**

A base class alone is not enough for interoperability. Two plugins could define
different signal classes. I suggest one shared schema so that an independent
detector does not need to import our planning plugin.

**Why separate capabilities?**

Object detection does not necessarily provide signal states or crossing
occupancy. The planner should check the specific data it needs. That checks
configuration; it does not guarantee that every frame contains a valid result.

**Can other domains extend it?**

Yes. Plugins can add marker subclasses without extending a global enum of all
object types. The core only defines the common metadata and agreed shared types.

**How would several detectors work together?**

For the first version, I suggest one final publisher. A fusion stage can combine
several detectors before publishing a snapshot, instead of letting them overwrite
each other's results. That composition policy still needs implementation.

**Does this replace the HD map?**

No. The map can provide geometry and route applicability. Perception provides
current observations, such as a light's state or an occupied crossing. Associating
an observation with the correct lane or map feature remains explicit.

**What is already done?**

The plugin has local observation classes, capability checks, capture-time
propagation, separate provenance and map references, with regression tests in
main. Sensor profiles reject truth-origin evidence; conflicting observations
for one map feature become unknown. Hardware clock synchronization is still an
adapter responsibility. Canonical core types, multi-producer composition and
ROS transport remain separate work after agreement. These tests do not establish
real-world recognition accuracy.

## Closing question

Would you prefer the common road-message types in a small optional core module,
alongside the generic observation types?
