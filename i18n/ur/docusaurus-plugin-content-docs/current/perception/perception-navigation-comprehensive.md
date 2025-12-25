---
title: "جامع ادراک اور نیویگیشن گائیڈ"
module: "perception"
word_count: 1200
learning_objectives:
  - "ادراک اور نیویگیشن کے تصورات کو سمجھنا"
  - "بصری SLAM سسٹم نافذ کرنا"
  - "ماحول کے نقشہ کاری اور مقام کاری تخلیق کرنا"
  - "راہ کی منصوبہ بندی الگورتھم تیار کرنا"
  - "نیویگیشن اور رکاوٹوں سے بچاؤ نافذ کرنا"
  - "ادراک کو مینوپولیشن کے ساتھ ضم کرنا"
  - "روبوٹکس کے لیے مضبوط سیکھنے کو سمجھنا"
prerequisites:
  - "ROS 2 کا علم"
  - "کمپیوٹر وژن کے بنیادیات کی سمجھ"
  - "پروگرامنگ کا تجربہ"
references:
  - "@durrant2006simultaneous"
  - "@bailey2006simultaneous"
  - "@mukadam2023nvblox"
validation_status: draft
---

# جامع ادراک اور نیویگیشن گائیڈ

## ادراک اور نیویگیشن کا تعارف

ادراک اور نیویگیشن خود مختار روبوٹس کے لیے بنیادی صلاحیتیں ہیں، جو انہیں اپنے ماحول کو سمجھنے اور اس کے اندر محفوظ طریقے سے منتقل ہونے کے قابل بناتی ہیں۔ ان سسٹم میں شامل ہیں:

- ماحولیاتی حس اور سمجھ
- نقشہ کاری اور مقام کاری
- راستہ کی منصوبہ بندی اور رکاوٹوں سے بچاؤ
- مینوپولیشن سسٹم کے ساتھ انضمام

## بصری SLAM نفاذ

ایک وقت میں مقام کاری اور نقشہ کاری (SLAM) روبوٹس کو نامعلوم ماحول کے نقشے تیار کرنے کی اجازت دیتا ہے جبکہ ان نقشوں کے اندر ایک ہی وقت میں ان کی پوزیشن کو ٹریک کرتا ہے۔ بصری SLAM خاص طور پر کیمرہ ڈیٹا کا استعمال کرتا ہے۔

### بصری SLAM کے اہم اجزاء

- فیچر کا پتہ لگانا اور مطابقت
- پوز اسٹیمیٹ
- نقشہ سازی
- لوپ کلاز ڈیٹیکشن
- بندل ایڈجسٹمنٹ

### ROS 2 SLAM انضمام

```python
# SLAM نوڈ کی مثال
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
        # SLAM کے لیے تصویر کی پروسیسنگ
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

## ماحول کی نقشہ کاری اور مقام کاری

### اوقوپنسی گرڈ نقوش

اوقوپنسی گرڈ نقوش ماحول کو ایک 2D گرڈ کے طور پر ظاہر کرتے ہیں جہاں ہر سیل میں قبضہ کرنے کا امکان ہوتا ہے:

```python
import numpy as np

class OccupancyGrid:
    def __init__(self, width, height, resolution):
        self.width = width
        self.height = height
        self.resolution = resolution
        self.grid = np.zeros((height, width))  # 0 = نامعلوم، 1 = قبضہ شدہ، 0.5 = مفت
    def update_cell(self, x, y, prob):
        if 0 <= x < self.width and 0 <= y < self.height:
            self.grid[y, x] = prob
```

### مقام کاری کی تکنیکیں

- AMCL (ایڈاپٹیو مونٹی کارلو مقام کاری)
- پارٹیکل فلٹرز
- کیلمین فلٹرز
- بصری-انرٹیل اوڈومیٹری

## راستہ کی منصوبہ بندی الگورتھم

### A* الگورتھم نفاذ

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
                grid[neighbor[0]][neighbor[1]] == 0):  # قبضہ نہیں ہے

                tentative_g = g_score[current] + 1

                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = tentative_g + heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))

    return []  # کوئی راستہ نہیں ملا
```

### نیویگیشن2 انضمام

نیویگیشن2 ROS 2 کے لیے ایک مکمل نیویگیشن اسٹیک فراہم کرتا ہے:

```xml
<!-- نیویگیشن2 لانچ فائل -->
<launch>
  <node pkg="nav2_bringup" exec="bringup_launch.py" name="nav2_bringup">
    <param name="use_sim_time" value="true"/>
  </node>
</launch>
```

## نیویگیشن اور رکاوٹوں سے بچاؤ

### مقامی منصوبہ ساز

مقامی منصوبہ ساز فوری رکاوٹوں سے بچاؤ کا انتظام کرتا ہے:

- ٹریجکٹری جنریشن
- کولیژن چیکنگ
- رفتار کنٹرول
- بحالی کے رویے

### عالمی منصوبہ ساز

عالمی منصوبہ ساز مجموعی راستہ کا حساب لگاتا ہے:

- اسٹیٹک میپ کا استعمال
- راستہ کی بہترین کارکردگی
- ضرورت پڑنے پر دوبارہ منصوبہ بندی

## ادراک اور مینوپولیشن کے تصورات

### آبجیکٹ ڈیٹیکشن انضمام

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

### گریسپ منصوبہ بندی

گریسپ منصوبہ بندی یہ طے کرنے میں ملوث ہے کہ اشیاء کو کیسے مینوپولیٹ کیا جائے:

- آبجیکٹ پوز اسٹیمیٹ
- گریسپ پوائنٹ کیلکولیشن
- ٹریجکٹری منصوبہ بندی
- فورس کنٹرول

## مضبوط سیکھنے کا انضمام

مضبوط سیکھنے نیویگیشن اور مینوپولیشن کو بہتر بنا سکتا ہے:

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

## NVIDIA Isaac انضمام

NVIDIA Isaac GPU تیز کاری کے ادراک کی صلاحیتیں فراہم کرتا ہے:

- ادراک کے لیے Isaac ROS
- سیمیولیشن کے لیے Isaac Sim
- GPU تیز کاری کا استنتاج
- CUDA کی بہترین کارکردگی

## ادراک اور نیویگیشن کے لیے بہترین طریقے

1. مضبوط ادراک کے لیے متعدد سینسرز کا استعمال کریں
2. مناسب سینسر فیوژن نافذ کریں
3. متنوع ماحول میں ٹیسٹ کریں
4. کمپیوٹیشنل پابندیوں پر غور کریں
5. حفاظتی چیکس اور فال بیکس نافذ کریں
6. کارکردگی کے معیارات کی توثیق کریں

## خلاصہ

ادراک اور نیویگیشن سسٹم خود مختار روبوٹکس کی بنیاد ہیں، جو روبوٹس کو اپنے ماحول کو سمجھنے اور محفوظ طریقے سے نیویگیٹ کرنے کے قابل بناتے ہیں۔ ان سسٹم کو قابل اعتماد آپریشن کو یقینی بنانے کے لیے متعدد ٹیکنالوجیز کا خیال شدہ انضمام اور جامع ٹیسٹنگ کی ضرورت ہوتی ہے۔