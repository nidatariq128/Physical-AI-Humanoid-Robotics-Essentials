---
title: "دليل شامل للإدراك والتنقل"
module: "perception"
word_count: 1200
learning_objectives:
  - "فهم مفاهيم الإدراك والتنقل"
  - "تنفيذ أنظمة SLAM البصرية"
  - "إنشاء رسم خرائط البيئة والتحديد المكاني"
  - "تطوير خوارزميات تخطيط المسار"
  - "تنفيذ التنقل وتجنب العوائق"
  - "دمج الإدراك مع التحكم"
  - "فهم التعلم المعزز للروبوتات"
prerequisites:
  - "معرفة بـ ROS 2"
  - "فهم أساسي لرؤية الحاسوب"
  - "تجربة في البرمجة"
references:
  - "@durrant2006simultaneous"
  - "@bailey2006simultaneous"
  - "@mukadam2023nvblox"
validation_status: draft
---

# دليل شامل للإدراك والتنقل

## مقدمة في الإدراك والتنقل

الإدراك والتنقل هما قدرات أساسية للروبوتات المستقلة، مما يمكّنها من فهم بيئتها والتحرك بأمان داخلها. تشمل هذه الأنظمة:

- إدراك البيئة وفهمها
- رسم الخرائط والتحديد المكاني
- تخطيط المسار وتجنب العوائق
- الدمج مع أنظمة التحكم

## تنفيذ SLAM البصري

يسمح التعيين والتحديد المكاني المتزامن (SLAM) للروبوتات ببناء خرائط للبيئات المجهولة أثناء تتبع موضعها في الوقت نفسه داخل هذه الخرائط. يستخدم SLAM البصري بشكل خاص بيانات الكاميرا.

### المكونات الأساسية لـ SLAM البصري

- اكتشاف الميزات ومطابقتها
- تقدير الموقف
- بناء الخريطة
- اكتشاف إغلاق الحلقة
- تعديل الحزمة

### دمج SLAM مع ROS 2

```python
# مثال على عقدة SLAM
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped

class VisualSLAMNode(Node):
    def __init__(self):
        super().__init__('visual_slam_node')
        self.subscription = self.create_subscription(
            Image,
            'camera/image_raw',
            self.image_callback,
            10)
        self.publisher = self.create_publisher(PoseStamped, 'slam/pose', 10)

    def image_callback(self, msg):
        # معالجة الصورة لـ SLAM
        pass

def main(args=None):
    rclpy.init(args=args)
    slam_node = VisualSLAMNode()
    rclpy.spin(slam_node)
    slam_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## رسم خرائط البيئة والتحديد المكاني

### خرائط الشبكة المحتلة

تمثل خرائط الشبكة المحتلة البيئة كشبكة ثنائية الأبعاد حيث تحتوي كل خلية على احتمال أن تكون محتلة:

```python
import numpy as np

class OccupancyGrid:
    def __init__(self, width, height, resolution):
        self.width = width
        self.height = height
        self.resolution = resolution
        self.grid = np.zeros((height, width))  # 0 = غير معروف، 1 = محتل، 0.5 = حر
    def update_cell(self, x, y, prob):
        if 0 <= x < self.width and 0 <= y < self.height:
            self.grid[y, x] = prob
```

### تقنيات التحديد المكاني

- AMCL (التحديد المكاني مونتي كارلو التكيفي)
- مرشحات الجسيمات
- مرشحات كالمان
- عداد المسافات البصري-العنصري

## خوارزميات تخطيط المسار

### تنفيذ خوارزمية A*

```python
import heapq

def a_star(grid, start, goal):
    def heuristic(a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    open_set = [(0, start)]
    came_from = {}
    g_score = {start: 0}
    f_score = {start: heuristic(start, goal)}

    while open_set:
        current = heapq.heappop(open_set)[1]

        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            return path[::-1]

        for dx, dy in [(0,1), (1,0), (0,-1), (-1,0)]:
            neighbor = (current[0] + dx, current[1] + dy)

            if (0 <= neighbor[0] < len(grid) and
                0 <= neighbor[1] < len(grid[0]) and
                grid[neighbor[0]][neighbor[1]] == 0):  # غير محتل

                tentative_g = g_score[current] + 1

                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = tentative_g + heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))

    return []  # لا يوجد مسار
```

### دمج Navigation2

يوفر Navigation2 مجموعة تنقل كاملة لـ ROS 2:

```xml
<!-- ملف تشغيل Navigation2 -->
<launch>
  <node pkg="nav2_bringup" exec="bringup_launch.py" name="nav2_bringup">
    <param name="use_sim_time" value="true"/>
  </node>
</launch>
```

## التنقل وتجنب العوائق

### المخطط المحلي

يتعامل المخطط المحلي مع تجنب العوائق الفوري:

- إنشاء المسار
- التحقق من التصادم
- التحكم في السرعة
- سلوكيات الاستعادة

### المخطط العام

يحسب المخطط العام المسار الشامل:

- استخدام الخريطة الثابتة
- تحسين المسار
- إعادة التخطيط عند الحاجة

## مفاهيم الإدراك والتحكم

### دمج اكتشاف الكائنات

```python
import cv2
import numpy as np

class PerceptionManipulation:
    def __init__(self):
        self.object_detector = cv2.dnn.readNetFromDarknet('config.cfg', 'weights.weights')

    def detect_objects(self, image):
        blob = cv2.dnn.blobFromImage(image, 1/255.0, (416, 416), swapRB=True, crop=False)
        self.object_detector.setInput(blob)
        outputs = self.object_detector.forward()
        return outputs
```

### تخطيط الإمساك

يتضمن تخطيط الإمساك تحديد كيفية التحكم في الكائنات:

- تقدير موضع الكائن
- حساب نقاط الإمساك
- تخطيط المسار
- التحكم في القوة

## دمج التعلم المعزز

يمكن أن يحسن التعلم المعزز من التنقل والتحكم:

```python
import numpy as np

class RobotRLAgent:
    def __init__(self, state_size, action_size):
        self.state_size = state_size
        self.action_size = action_size
        self.q_table = np.zeros((state_size, action_size))
        self.learning_rate = 0.1
        self.discount_factor = 0.95
        self.epsilon = 0.1

    def choose_action(self, state):
        if np.random.rand() <= self.epsilon:
            return np.random.choice(self.action_size)
        return np.argmax(self.q_table[state, :])

    def learn(self, state, action, reward, next_state):
        td_target = reward + self.discount_factor * np.max(self.q_table[next_state, :])
        td_error = td_target - self.q_table[state, action]
        self.q_table[state, action] += self.learning_rate * td_error
```

## دمج NVIDIA Isaac

يوفر NVIDIA Isaac قدرات إدراك مسرعة ب_GPU:

- Isaac ROS للإدراك
- Isaac Sim للimulation
- استنتاج مسرّع ب_GPU
- تحسين CUDA

## أفضل الممارسات للإدراك والتنقل

1. استخدام أجهزة استشعار متعددة للإدراك القوي
2. تنفيذ دمج أجهزة الاستشعار بشكل صحيح
3. الاختبار في بيئات متنوعة
4. أخذ قيود الحوسبة في الاعتبار
5. تنفيذ فحوصات الأمان وبدائل الطوارئ
6. التحقق من معايير الأداء

## ملخص

تشكل أنظمة الإدراك والتنقل قاعدة الروبوتات المستقلة، مما يمكّن الروبوتات من فهم بيئتها والتنقل بأمان. تتطلب هذه الأنظمة دمجًا دقيقًا لتقنيات متعددة واختبارًا شاملاً لضمان التشغيل الموثوق.