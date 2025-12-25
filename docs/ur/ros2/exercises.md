---
title: "ROS 2 مشقیں"
module: "ros2"
word_count: 320
learning_objectives:
  - "ROS 2 کے بنیادی کمانڈز استعمال کرنا"
  - "ROS 2 نوڈس بنانا اور چلانا"
  - "ROS 2 ٹوپکس کے ذریعے کمیونیکیشن کرنا"
prerequisites:
  - "ROS 2 انسٹال ہو"
  - "ROS 2 کے بنیادی تصورات"
references:
  - "@ros2_documentation"
validation_status: draft
---

# ROS 2 مشقیں

## مشق 1: ROS 2 ورک اسپیس کا سیٹ اپ

### اہداف:
- ROS 2 ورک اسپیس بنانا
- ROS 2 این وائرمنٹ سیٹ کرنا
- بنیادی ROS 2 کمانڈز استعمال کرنا

### اسٹیپس:
1. نیا ڈائریکٹری بنائیں:
   ```bash
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws
   ```

2. ورک اسپیس کو سیٹ اپ کریں:
   ```bash
   colcon build
   source install/setup.bash
   ```

## مشق 2: ROS 2 نوڈ بنانا

### اہداف:
- ROS 2 نوڈ لکھنا
- نوڈ کو کمپائل کرنا
- نوڈ کو چلانا

### ہدایات:
1. نیا پیکج بنائیں:
   ```bash
   cd ~/ros2_ws/src
   ros2 pkg create --build-type ament_python my_robot_node --dependencies rclpy
   ```

2. نوڈ کو لکھیں (my_robot_node/my_robot_node/main.py)

3. نوڈ کو چلائیں:
   ```bash
   ros2 run my_robot_node main
   ```

## مشق 3: ٹوپکس کے ذریعے کمیونیکیشن

### اہداف:
- ٹوپکس کو سبسکرائب کرنا
- ٹوپکس میں میسج بھیجنا
- ڈیٹا کو فلٹر کرنا

### ہدایات:
1. موجودہ ٹوپکس دیکھیں:
   ```bash
   ros2 topic list
   ```

2. ٹوپک کا امتحان کریں:
   ```bash
   ros2 topic echo /topic_name
   ```

3. ٹوپک میں میسج بھیجیں:
   ```bash
   ros2 topic pub /topic_name std_msgs/String "data: 'Hello'"
   ```

## مشق 4: سروسز کے ذریعے کمیونیکیشن

### اہداف:
- سروس کال کرنا
- سروس کو فراہم کرنا
- کسٹم میسجز استعمال کرنا

### ہدایات:
1. سروس کال کریں:
   ```bash
   ros2 service call /service_name service_type
   ```

2. سروس کی فراہمی کریں:
   ```bash
   ros2 run package_name service_server
   ```

## مشق 5: ٹی ایف 2 استعمال کرنا

### اہداف:
- ٹرنسفرمیشن کو شائع کرنا
- ٹرنسفرمیشن کو حاصل کرنا
- کوآرڈینیٹ فریم کو تبدیل کرنا

### ہدایات:
1. ٹی ایف 2 کو انسٹال کریں:
   ```bash
   sudo apt install ros-humble-tf2-tools
   ```

2. ٹی ایف ٹری کو دیکھیں:
   ```bash
   ros2 run tf2_tools view_frames
   ```

## مشق 6: کنٹرولرز کو منظم کرنا

### اہداف:
- کنٹرولر لانچ کرنا
- کنٹرولر کو سوئچ کرنا
- کنٹرولر کی ترتیبات کو تبدیل کرنا

### ہدایات:
1. کنٹرولر منیجر لانچ کریں:
   ```bash
   ros2 run controller_manager spawner controller_name
   ```

2. کنٹرولر کو ڈسپلے کریں:
   ```bash
   ros2 control list_controllers
   ```

## خلاصہ

یہ مشقیں ROS 2 کے بنیادی خصوصیات کو سمجھنے اور استعمال کرنے کے لیے ضروری ہیں۔ ہر مشق کو کامیابی کے ساتھ مکمل کرنا چاہیے تاکہ ROS 2 کے بنیادی تصورات کو سمجھا جا سکے۔