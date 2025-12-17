---
title: Chapter 02 Using OpenAI Whisper for voice command
---

OpenAI Whisper is a state-of-the-art speech-to-text (STT) model that can be integrated into robots to enable voice command recognition. By converting spoken language into text, Whisper allows robots to understand natural language instructions and act accordingly.

## **Using OpenAI Whisper for Voice Commands in Robotics** 🎤🤖

**OpenAI Whisper** is a **state-of-the-art speech-to-text (STT) model** that can be integrated into robots to enable **voice command recognition**. By converting spoken language into text, Whisper allows robots to **understand natural language instructions** and act accordingly.

---

## 🧠 How Whisper Fits in Robotics

1. **Audio Input**

   * Microphone captures human speech.
2. **Speech-to-Text Conversion (Whisper)**

   * Converts raw audio into text.
3. **Natural Language Processing**

   * Optional: feed text into **LLM** (like ChatGPT) to interpret commands.
4. **Command Execution**

   * Robot middleware (ROS 2, rclpy) translates commands into low-level actions.
5. **Feedback to User**

   * Robot can confirm actions via audio or visual output.

---

## 🔁 Workflow

```
[Microphone] → Audio Signal
       ↓
[OpenAI Whisper] → Text Output
       ↓
[Command Parser / LLM] → Structured Robot Action
       ↓
[ROS 2 Node / Controller] → Robot Motion / Action
       ↓
[Robot Executes Task] → Feedback to User
```

---

## 🔧 Implementation Example (Python + ROS 2)

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import whisper
import sounddevice as sd
import numpy as np

class VoiceControlNode(Node):
    def __init__(self):
        super().__init__('voice_control')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.model = whisper.load_model("base")
        self.samplerate = 16000

    def listen_and_execute(self, duration=5):
        audio = sd.rec(int(duration * self.samplerate), samplerate=self.samplerate, channels=1)
        sd.wait()
        audio = audio.flatten()
        result = self.model.transcribe(audio)
        command = result['text']
        self.get_logger().info(f"Recognized command: {command}")

        twist = Twist()
        if "forward" in command:
            twist.linear.x = 0.2
        elif "backward" in command:
            twist.linear.x = -0.2
        elif "left" in command:
            twist.angular.z = 0.5
        elif "right" in command:
            twist.angular.z = -0.5
        self.publisher.publish(twist)

rclpy.init()
node = VoiceControlNode()
while True:
    node.listen_and_execute()
```

---

## 🧪 Use Cases

* **Mobile robot navigation** via voice commands
* **Industrial robots** – hands-free control on factory floor
* **Assistive robots** – home automation, eldercare, accessibility
* **Human-robot collaboration** – conversational task execution

---

## ✅ Advantages of Whisper

* **Multilingual support**
* **High accuracy** even in noisy environments
* **Lightweight deployment** on local hardware or edge devices
* **Open-source and customizable**

---

## 📌 Summary

Using **OpenAI Whisper for voice commands** enables robots to **understand and act on natural language instructions**, bridging human communication with robotic action.
When combined with **ROS 2 and LLMs**, it creates **intuitive, voice-controlled autonomous systems** 🤖🎤
