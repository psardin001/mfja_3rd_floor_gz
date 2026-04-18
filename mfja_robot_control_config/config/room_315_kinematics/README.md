# Room 315 Kinematic Shuttle Guide

هذا الدليل يشرح نظام حركة الشاتل الكينماتيكي لغرفة 315، وكيف نشغله سواء في محاكاة غرفة 315 فقط أو في محاكاة الطابق الكامل.

الفكرة الأساسية أن الشاتل لا يتحرك بفيزياء العجلات والاحتكاك، بل يتم تحديث وضعه مباشرة على السكة باستخدام `set_pose`. هذا مقصود حاليًا حتى نثبت منطق الشبكة والسويتشات والتصادمات قبل الدخول في contact dynamics.

## ماذا يوجد هنا

- `rail_network.yaml`: شبكة السكك كـ directed graph، فيها nodes وsegments وswitches وfixed transitions.
- `CSV/`: ملفات المسارات الأصلية لكل segment.
- `normalized_segments/`: ملفات CSV بعد إزالة النقاط المكررة وإعادة الفهرسة.
- `segment_summary.yaml`: ملخص preprocessing.
- `validation_report.yaml`: تقرير التحقق من الأطوال، السناب، الفجوات، واتجاهات التماس.
- `debug_plots/`: صور فحص بصرية للمسارات والشبكة.
- `room_315_kinematic_shuttle_node.py`: نود ROS 2 التي تحرك شاتل واحد أو أكثر داخل Gazebo.

كل segment باتجاه واحد فقط: من `index=0` إلى آخر index في ملف الـ CSV. إذا وصل الشاتل لنهاية segment ولا يوجد successor صالح حسب حالة السويتشات، يدخل وضع `FALLING` بدل أن يصحح نفسه بصمت.

## بناء المشروع

نفذ هذا مرة بعد أي تعديل بالكود أو ملفات install:

```bash
cd /home/tiago/ALI_ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --base-paths \
  src/mfja_3rd_floor_gz/mfja_robot_control_config \
  src/mfja_3rd_floor_gz/mfja_3rd_floor_description \
  src/mfja_3rd_floor_gz/mfja_room_315_bringup \
  src/mfja_3rd_floor_gz/mfja_3rd_floor_bringup \
  src/mfja_3rd_floor_gz/mfja_3rd_floor_gz \
  --packages-select \
  mfja_robot_control_config \
  mfja_3rd_floor_description \
  mfja_room_315_bringup \
  mfja_3rd_floor_bringup \
  mfja_3rd_floor_gz \
  --symlink-install
source install/setup.bash
```

إذا ظهر لك أن package غير معروف عند البناء، استخدم أمر `--base-paths` أعلاه كما هو، لأن هذا الـ repository يحتوي packages داخلية وليس package واحد فقط.

إذا عدلت ملفات README فقط، لا تحتاج إعادة بناء.

## تشغيل غرفة 315 فقط

افتح Terminal 1 وشغل Gazebo لغرفة 315:

```bash
cd /home/tiago/ALI_ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch mfja_room_315_bringup room_315_only.launch.py \
  robots:=none \
  start_paused:=false \
  gui:=true
```

افتح Terminal 2 وشغل نود الشاتل على عالم `room_315_only`:

```bash
cd /home/tiago/ALI_ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 run mfja_robot_control_config room_315_kinematic_shuttle_node.py --ros-args \
  -p gazebo_world_name:=room_315_only \
  -p start_slot:=2 \
  -p enable_gazebo_set_pose:=true \
  -p enable_gazebo_spawn:=true \
  -p speed:=0.2 \
  -p gazebo_set_pose_rate_hz:=10.0
```

## تشغيل الطابق الكامل

مهم جدًا: اسم ملف العالم هو `mfja_3rd_floor.world`، ويجب أن يكون اسم العالم داخل الملف أيضًا:

```xml
<world name="mfja_3rd_floor">
```

لهذا السبب نستخدم مع النود:

```bash
-p gazebo_world_name:=mfja_3rd_floor
```

إذا كان العالم مفتوحًا قبل تعديل الاسم الداخلي، أطفئ Gazebo وافتحه من جديد. الخدمات لا تتغير داخل جلسة Gazebo القديمة.

افتح Terminal 1 وشغل Gazebo للطابق الكامل:

```bash
cd /home/tiago/ALI_ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch mfja_3rd_floor_bringup full_floor.launch.py \
  robots:=none \
  start_paused:=false \
  gui:=true
```

افتح Terminal 2 وشغل نفس نود الشاتل، لكن غيّر فقط اسم العالم إلى `mfja_3rd_floor`:

```bash
cd /home/tiago/ALI_ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 run mfja_robot_control_config room_315_kinematic_shuttle_node.py --ros-args \
  -p gazebo_world_name:=mfja_3rd_floor \
  -p start_slot:=2 \
  -p enable_gazebo_set_pose:=true \
  -p enable_gazebo_spawn:=true \
  -p speed:=0.2 \
  -p gazebo_set_pose_rate_hz:=10.0
```

إذًا نفس الخصائص تعمل في الحالتين. الفرق فقط:

- غرفة 315 فقط: `ros2 launch mfja_room_315_bringup room_315_only.launch.py` و `-p gazebo_world_name:=room_315_only`
- الطابق الكامل: `ros2 launch mfja_3rd_floor_bringup full_floor.launch.py` و `-p gazebo_world_name:=mfja_3rd_floor`

## التأكد من خدمات Gazebo

الـ launch يشغل bridge تلقائيًا للخدمتين:

- `/world/<world_name>/set_pose`
- `/world/<world_name>/create`

افحص الخدمات من Terminal جديد:

```bash
cd /home/tiago/ALI_ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 service list | grep -E "set_pose|create"
```

في غرفة 315 يجب أن ترى:

```text
/world/room_315_only/set_pose
/world/room_315_only/create
```

في الطابق الكامل يجب أن ترى:

```text
/world/mfja_3rd_floor/set_pose
/world/mfja_3rd_floor/create
```

إذا ظهرت بدل ذلك خدمات مثل `/world/default/set_pose`، فهذا يعني أن العالم المفتوح ما زال اسمه الداخلي `default`، أو أنك لم تعد تشغيل Gazebo بعد تعديل ملف العالم. أطفئ Gazebo، أعد البناء عند الحاجة، ثم شغل launch من جديد.

إذا ظهرت رسالة `Gazebo set_pose service is not ready yet`، غالبًا `gazebo_world_name` لا يطابق العالم المفتوح، أو Gazebo لم يفتح بعد، أو الـ bridge غير شغال.

## أماكن انطلاق الشاتل

يوجد فقط 4 أماكن انطلاق مسموحة:

| slot | pose في Gazebo |
| --- | --- |
| `1` | `-14.95 -3.86 0.84 0 0 3.14` |
| `2` | `-15.43 -3.86 0.84 0 0 3.14` |
| `3` | `-15.24 -5.54 0.84 0 0 0` |
| `4` | `-14.77 -5.54 0.84 0 0 0` |

تشغيل شاتل واحد من slot محدد:

```bash
ros2 run mfja_robot_control_config room_315_kinematic_shuttle_node.py --ros-args \
  -p gazebo_world_name:=room_315_only \
  -p start_slot:=3 \
  -p enable_gazebo_set_pose:=true \
  -p enable_gazebo_spawn:=true \
  -p speed:=0.2 \
  -p gazebo_set_pose_rate_hz:=10.0
```

إذا كان التشغيل على الطابق الكامل، بدّل فقط:

```bash
-p gazebo_world_name:=mfja_3rd_floor
```

## تشغيل عدة شاتلات من البداية

تشغيل 4 شاتلات من الأماكن الأربعة:

```bash
ros2 run mfja_robot_control_config room_315_kinematic_shuttle_node.py --ros-args \
  -p gazebo_world_name:=room_315_only \
  -p shuttle_count:=4 \
  -p start_slots:=1,2,3,4 \
  -p enable_gazebo_set_pose:=true \
  -p enable_gazebo_spawn:=true \
  -p speed:=0.2 \
  -p gazebo_set_pose_rate_hz:=10.0
```

نفس الأمر للطابق الكامل:

```bash
ros2 run mfja_robot_control_config room_315_kinematic_shuttle_node.py --ros-args \
  -p gazebo_world_name:=mfja_3rd_floor \
  -p shuttle_count:=4 \
  -p start_slots:=1,2,3,4 \
  -p enable_gazebo_set_pose:=true \
  -p enable_gazebo_spawn:=true \
  -p speed:=0.2 \
  -p gazebo_set_pose_rate_hz:=10.0
```

إذا طلبت `shuttle_count` أكبر من عدد `start_slots`، النود يكرر slots بالترتيب. لا يوجد حد أعلى برمجيًا لعدد الشاتلات، لكن الأداء العملي تابع لقدرة Gazebo والجهاز.

## إضافة شاتل أثناء التشغيل

افتح Terminal جديد بعد تشغيل Gazebo ونود الشاتل، ثم أرسل أمر إضافة شاتل:

```bash
cd /home/tiago/ALI_ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 topic pub --once /room_315/shuttle/add_cmd std_msgs/msg/String "{data: 'slot=3'}"
```

إضافة شاتل باسم محدد وسرعة محددة:

```bash
ros2 topic pub --once /room_315/shuttle/add_cmd std_msgs/msg/String "{data: 'entity=room315_shuttle_5 slot=3 speed=0.2'}"
```

إضافة شاتل بالـ slot فقط، والنود يختار الاسم تلقائيًا:

```bash
ros2 topic pub --once /room_315/shuttle/add_cmd std_msgs/msg/String "{data: 'slot=1'}"
```

أو اختصارًا:

```bash
ros2 topic pub --once /room_315/shuttle/add_cmd std_msgs/msg/String "{data: '4'}"
```

ملاحظات مهمة:

- أول 4 شاتلات موجودة مسبقًا في العالم ومخفية، والنود يحركها عند الحاجة.
- الشاتل الخامس وما بعده يتم إنشاؤه عبر خدمة `/world/<world_name>/create`.
- يمكن إضافة أكثر من شاتل من نفس slot، لكن إذا كان المكان مشغولًا فالتصادم/الاقتراب سيجعل الشاتل يتوقف.
- لا يوجد block occupancy كامل بعد، الموجود حاليًا هو تجنب تصادم بالمسافة بين مراكز الشاتلات.

## تجنب التصادم بين الشاتلات

تجنب التصادم مفعل افتراضيًا:

- `enable_collision_avoidance=true`
- `shuttle_collision_distance_m=0.33`

طول ملف الشاتل المقاس تقريبًا `0.343 m`، لذلك القيمة الافتراضية `0.33 m` مناسبة كبداية. عندما يقترب شاتل من شاتل آخر لمسافة أقل من هذه القيمة، يتحول الشاتل القادم إلى `WAITING` ويقف عند آخر موضع آمن بدل أن يندمج مع الشاتل الآخر.

يمكنك تعديل المسافة عند التشغيل إذا احتجت:

```bash
ros2 run mfja_robot_control_config room_315_kinematic_shuttle_node.py --ros-args \
  -p gazebo_world_name:=room_315_only \
  -p start_slot:=2 \
  -p enable_gazebo_set_pose:=true \
  -p enable_gazebo_spawn:=true \
  -p enable_collision_avoidance:=true \
  -p shuttle_collision_distance_m:=0.33 \
  -p speed:=0.2 \
  -p gazebo_set_pose_rate_hz:=10.0
```

ولأن القيمتين مفعّلتان افتراضيًا، عادة لا تحتاج كتابتهما في التيرمنال.

## مراقبة حالة الشاتلات

الحالة العامة تنشر على:

```text
/room_315/shuttle/state
```

افحصها:

```bash
ros2 topic echo /room_315/shuttle/state --once
```

ستجد داخل JSON:

- `shuttle_count`: عدد الشاتلات.
- `shuttles`: قائمة بكل شاتل، segment الحالي، `s`، السرعة، وpose في Gazebo.
- `switch_states`: وضع السويتشات الحالي.
- `blocked_by`: اسم الشاتل الذي يوقف هذا الشاتل عند التصادم، أو `null`.
- `collision_distance_m`: المسافة الفعلية عند التوقف، أو `null`.
- `mode`: واحدة من `MOVING`, `WAITING`, `FALLING`.

موضع الشاتل الأول ينشر أيضًا على:

```bash
ros2 topic echo /room_315/shuttle/pose_cmd --once
```

ومواضع الشاتلات الأخرى تنشر تحت:

```text
/room_315/shuttles/<entity_name>/pose_cmd
```

مثال:

```bash
ros2 topic echo /room_315/shuttles/room315_shuttle_3/pose_cmd --once
```

## التحكم بالسويتشات

أرسل أوامر السويتشات إلى:

```text
/room_315/switch_states
```

حالات السويتش:

- `G` أو `GRAND_BOUCLE` أو `BIG` أو `LARGE`: الحلقة الكبيرة.
- `S` أو `PETIT_BOUCLE` أو `SMALL`: الحلقة الصغيرة.

السويتشات المنطقية هي:

- `A1`
- `A2`
- `A3`
- `A4`

يمكن أيضًا استخدام أسماء مرئية يمين/يسار:

- `A1R`, `A2R`, `A3R`, `A4R`
- `A1L`, `A2L`, `A3L`, `A4L`

عند استخدام `A1R` أو `A1L`، النود يحدث منطق المسار للسويتش `A1` ويرسل أيضًا أمرًا مرئيًا لتحريك مجسم السويتش في Gazebo.

تبديل كل السويتشات إلى الحلقة الكبيرة:

```bash
ros2 topic pub --once /room_315/switch_states std_msgs/msg/String "{data: 'ALL=G'}"
```

تبديل كل السويتشات إلى الحلقة الصغيرة:

```bash
ros2 topic pub --once /room_315/switch_states std_msgs/msg/String "{data: 'ALL=S'}"
```

تبديل سويتش واحد:

```bash
ros2 topic pub --once /room_315/switch_states std_msgs/msg/String "{data: 'A1=G'}"
ros2 topic pub --once /room_315/switch_states std_msgs/msg/String "{data: 'A1=S'}"
ros2 topic pub --once /room_315/switch_states std_msgs/msg/String "{data: 'A2=G'}"
ros2 topic pub --once /room_315/switch_states std_msgs/msg/String "{data: 'A2=S'}"
ros2 topic pub --once /room_315/switch_states std_msgs/msg/String "{data: 'A3=G'}"
ros2 topic pub --once /room_315/switch_states std_msgs/msg/String "{data: 'A3=S'}"
ros2 topic pub --once /room_315/switch_states std_msgs/msg/String "{data: 'A4=G'}"
ros2 topic pub --once /room_315/switch_states std_msgs/msg/String "{data: 'A4=S'}"
```

تبديل بأسماء `RIGHT/LEFT` المرئية:

```bash
ros2 topic pub --once /room_315/switch_states std_msgs/msg/String "{data: 'A1R=G'}"
ros2 topic pub --once /room_315/switch_states std_msgs/msg/String "{data: 'A1R=S'}"
ros2 topic pub --once /room_315/switch_states std_msgs/msg/String "{data: 'A1L=G'}"
ros2 topic pub --once /room_315/switch_states std_msgs/msg/String "{data: 'A1L=S'}"
```

أوامر جماعية:

```bash
ros2 topic pub --once /room_315/switch_states std_msgs/msg/String "{data: 'A1=S A2=G A3=S A4=G'}"
ros2 topic pub --once /room_315/switch_states std_msgs/msg/String "{data: 'A1R=S A2R=S A3R=G A4R=G'}"
ros2 topic pub --once /room_315/switch_states std_msgs/msg/String "{data: 'RIGHT=G'}"
ros2 topic pub --once /room_315/switch_states std_msgs/msg/String "{data: 'LEFT=S'}"
```

مهم: الأفضل دائمًا أن ترسل إلى `/room_315/switch_states` وليس مباشرة إلى visual topic، لأن هذا يحدث منطق مسار الشاتل ويحدث شكل السويتش في Gazebo بنفس الوقت.

## حفظ/مزامنة حالة السويتشات بعد إعادة تشغيل النود

النود يستمع افتراضيًا إلى:

```text
/mfja/conveyor/switch_states
```

إذا كان visual switch controller شغالًا وينشر آخر حالة للسويتشات، النود الجديد يقرأ هذه الحالة عند التشغيل ويطابق منطق المسار معها. هذا يمنع مشكلة أن النود يرجع يعتبر كل السويتشات على الحالة الافتراضية بعد إعادة التشغيل.

لفحص الحالة المرئية الحالية:

```bash
ros2 topic echo /mfja/conveyor/switch_states --once
```

إذا أردت تعطيل المزامنة:

```bash
-p sync_from_visual_switch_states:=false
```

## معايرة المسار أثناء التشغيل

المسار الحالي تمت معايرته داخل ملفات CSV، لذلك غالبًا لا تحتاج `scale` أو `offset`. لكن إذا أردت تجربة انزياح أو scale أثناء التشغيل، استخدم:

```text
/room_315/shuttle/pose_offset_cmd
```

زيادة X بمقدار `0.01`:

```bash
ros2 topic pub --once /room_315/shuttle/pose_offset_cmd std_msgs/msg/String "{data: 'dx=0.01'}"
```

زيادة Y بمقدار `-0.02`:

```bash
ros2 topic pub --once /room_315/shuttle/pose_offset_cmd std_msgs/msg/String "{data: 'dy=-0.02'}"
```

ضبط offset مطلق:

```bash
ros2 topic pub --once /room_315/shuttle/pose_offset_cmd std_msgs/msg/String "{data: 'x=0.0 y=0.0 z=0.0'}"
```

ضبط scale مطلق:

```bash
ros2 topic pub --once /room_315/shuttle/pose_offset_cmd std_msgs/msg/String "{data: 'sx=1.0 sy=1.0'}"
```

زيادة scale تدريجيًا:

```bash
ros2 topic pub --once /room_315/shuttle/pose_offset_cmd std_msgs/msg/String "{data: 'dsx=0.01 dsy=-0.01'}"
```

تغيير أصل الـ scale:

```bash
ros2 topic pub --once /room_315/shuttle/pose_offset_cmd std_msgs/msg/String "{data: 'origin_x=-15.85 origin_y=-4.52'}"
```

إلغاء كل offset/scale التجريبي:

```bash
ros2 topic pub --once /room_315/shuttle/pose_offset_cmd std_msgs/msg/String "{data: 'reset'}"
```

يمكن أيضًا تمرير القيم عند تشغيل النود:

```bash
-p pose_scale_x:=1.0 \
-p pose_scale_y:=1.0 \
-p pose_offset_x:=0.0 \
-p pose_offset_y:=0.0 \
-p pose_offset_z:=0.0
```

## أدوات preprocessing والتحقق

بعد تعديل CSV أو `rail_network.yaml`:

```bash
cd /home/tiago/ALI_ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 run mfja_robot_control_config room_315_csv_preprocessor.py
ros2 run mfja_robot_control_config room_315_network_validator.py
ros2 run mfja_robot_control_config room_315_segment_plot.py
```

المخرجات الأساسية:

- `mfja_robot_control_config/config/room_315_kinematics/segment_summary.yaml`
- `mfja_robot_control_config/config/room_315_kinematics/validation_report.yaml`
- `mfja_robot_control_config/config/room_315_kinematics/debug_plots/network_validation.png`
- `mfja_robot_control_config/config/room_315_kinematics/debug_plots/room_315_segments_overview.png`

تشغيل core offline بدون Gazebo:

```bash
ros2 run mfja_robot_control_config room_315_kinematic_shuttle.py \
  --switch A1=G \
  --switch A2=G \
  --switch A3=G \
  --switch A4=G
```

## أهم parameters في نود الشاتل

| parameter | default | المعنى |
| --- | --- | --- |
| `gazebo_world_name` | `room_315_only` | اسم عالم Gazebo المستخدم لبناء `/world/<name>/set_pose` و`/world/<name>/create`. |
| `enable_gazebo_set_pose` | `false` | إذا `true` يحرك موديل الشاتل داخل Gazebo. |
| `enable_gazebo_spawn` | `true` | يسمح بإنشاء شاتلات جديدة عبر خدمة Gazebo create. |
| `start_slot` | `2` | مكان انطلاق الشاتل الواحد. |
| `start_slots` | empty | قائمة أماكن الانطلاق عند تشغيل عدة شاتلات، مثل `1,2,3,4`. |
| `shuttle_count` | `1` | عدد الشاتلات عند بدء النود. |
| `gazebo_entity_name` | `room315_shuttle_1` | اسم موديل الشاتل الواحد في Gazebo. |
| `gazebo_entity_names` | empty | قائمة أسماء عند تشغيل عدة شاتلات. |
| `preloaded_shuttle_count` | `4` | عدد الشاتلات الموجودة مسبقًا في العالم. |
| `speed` | `0.25` | سرعة الشاتل على المسار بالمتر/ثانية. |
| `update_rate_hz` | `30.0` | معدل تحديث الحساب الداخلي. |
| `gazebo_set_pose_rate_hz` | `10.0` | معدل إرسال `set_pose` إلى Gazebo. |
| `enable_collision_avoidance` | `true` | تفعيل توقف الشاتل قبل التصادم. |
| `shuttle_collision_distance_m` | `0.33` | أقل مسافة مسموحة بين مراكز الشاتلات. |
| `switch_command_topic` | `/room_315/switch_states` | topic أوامر السويتشات. |
| `add_shuttle_command_topic` | `/room_315/shuttle/add_cmd` | topic إضافة شاتل أثناء التشغيل. |
| `state_topic` | `/room_315/shuttle/state` | topic حالة الشاتلات. |
| `pose_offset_command_topic` | `/room_315/shuttle/pose_offset_cmd` | topic المعايرة أثناء التشغيل. |
| `publish_visual_switch_commands` | `true` | يجعل أوامر السويتش تحرك المجسمات المرئية أيضًا. |
| `sync_from_visual_switch_states` | `true` | مزامنة من آخر حالة مرئية منشورة. |

## أوامر فحص سريعة

فحص الخدمات:

```bash
ros2 service list | grep -E "set_pose|create"
```

فحص حالة الشاتلات:

```bash
ros2 topic echo /room_315/shuttle/state --once
```

فحص حالة السويتشات المرئية:

```bash
ros2 topic echo /mfja/conveyor/switch_states --once
```

فحص كل topics الخاصة بالغرفة:

```bash
ros2 topic list | grep room_315
```

## Troubleshooting

- إذا لم يفتح Gazebo: شغل launch الخاص بالغرفة أو الطابق في Terminal مستقل قبل تشغيل نود الشاتل.
- إذا ظهرت `set_pose service is not ready`: تأكد أن `gazebo_world_name` يطابق العالم المفتوح.
- إذا كنت تعمل على الطابق الكامل وظهرت `/world/default/set_pose`: أطفئ Gazebo وافتحه من جديد بعد التأكد أن `mfja_3rd_floor_description/worlds/mfja_3rd_floor.world` يحتوي `<world name="mfja_3rd_floor">`.
- إذا ظهرت `spawn service is not ready`: تأكد من وجود `/world/<world_name>/create` في `ros2 service list`.
- إذا الشاتل لا يظهر بعد إضافته: غالبًا خدمة `create` غير جاهزة أو اسم entity مكرر.
- إذا الشاتل توقف فجأة بوضع `WAITING`: غالبًا اقترب من شاتل آخر حسب `shuttle_collision_distance_m`.
- إذا الشاتل دخل `FALLING`: لا يوجد successor صالح في `rail_network.yaml` حسب وضع السويتشات الحالي.
- إذا السويتش تغير شكله ولم يتغير مسار الشاتل: استخدم `/room_315/switch_states` بدل إرسال أمر مباشر إلى `/mfja/conveyor/switch_cmd`.
- إذا مسار الشاتل يبدو منزاحًا: جرّب `pose_offset_cmd` مؤقتًا، ثم بعد معرفة القيم الصحيحة طبّق التصحيح على ملفات CSV.
