---
title: Chapter 2 - Voice-to-Action — Language Interfaces for Robots
sidebar_label: "Chapter 2: Voice-to-Action — Language Interfaces for Robots"
---

# Chapter 2: Voice-to-Action — Language Interfaces for Robots

## Introduction

This chapter focuses on implementing voice-to-action interfaces that enable robots to understand and respond to natural language commands. We'll explore OpenAI Whisper for speech recognition, techniques for translating voice commands into structured intents, and integration with ROS 2 communication patterns.

Building upon the theoretical foundation from Chapter 1, we now move to practical implementation of the language interface. This chapter demonstrates how to create robust human-robot interaction loops that can handle ambiguity, provide confirmation, and offer user feedback.

## Speech-to-Text Using OpenAI Whisper

### Introduction to OpenAI Whisper

OpenAI Whisper is a state-of-the-art automatic speech recognition (ASR) system that excels at transcribing speech in multiple languages. For robotics applications, Whisper offers several advantages:

- **Multilingual Support**: Capable of transcribing speech in numerous languages
- **Robustness**: Performs well in various acoustic environments
- **Accuracy**: High transcription accuracy across different speakers and accents
- **Open Source**: Available for self-hosting to maintain privacy and reduce latency

### Whisper Integration for Robotics

For robotic applications, Whisper can be integrated in several ways:

#### Real-time Streaming Approach
```python
import whisper
import pyaudio
import numpy as np

class RobotWhisperInterface:
    def __init__(self, model_size="base"):
        self.model = whisper.load_model(model_size)
        self.audio_buffer = []

    def process_audio_chunk(self, audio_data):
        """Process streaming audio and detect speech commands"""
        self.audio_buffer.extend(audio_data)

        # Process buffer periodically to detect speech
        if len(self.audio_buffer) > SAMPLE_RATE * 2:  # Process 2-second chunks
            audio_segment = np.array(self.audio_buffer)
            result = self.model.transcribe(audio_segment, fp16=False)

            # Reset buffer after processing
            self.audio_buffer = []
            return result['text']

        return None
```

#### Batch Processing Approach
For applications where real-time processing isn't critical:
```python
def transcribe_command(audio_file_path):
    """Transcribe a complete audio command"""
    model = whisper.load_model("base")
    result = model.transcribe(audio_file_path)
    return result['text']
```

### Optimizing Whisper for Robot Environments

Robot environments present unique challenges for speech recognition:

#### Noise Reduction
- **Acoustic Filtering**: Apply noise reduction algorithms before Whisper processing
- **Directional Microphones**: Use beamforming microphones to focus on speaker
- **Environmental Adaptation**: Train acoustic models on robot-specific environments

#### Latency Optimization
- **Model Selection**: Choose Whisper model size based on latency requirements
- **Hardware Acceleration**: Utilize GPU acceleration when available
- **Caching**: Cache frequently-used phrases for faster recognition

## Translating Voice Commands into Structured Intents

### Intent Recognition Pipeline

The process of converting voice commands into structured robotic intents involves several steps:

```
Voice Input → Speech-to-Text → Natural Language Processing → Intent Extraction → ROS 2 Commands
```

### Natural Language Processing for Robotics

Robotic applications require specialized NLP techniques that differ from general-purpose language understanding:

#### Command Structure Recognition
Robotic commands typically follow predictable patterns:
- **Action-Object-Location**: "Pick up the red ball from the table"
- **Navigation Commands**: "Go to the kitchen" or "Move to the left"
- **Manipulation Commands**: "Open the door" or "Pour the water"
- **Conditional Commands**: "If the light is red, stop"

#### Named Entity Recognition for Robotics
Identify key elements in commands:
- **Objects**: "ball", "cup", "door"
- **Colors**: "red", "blue", "green"
- **Locations**: "table", "kitchen", "left side"
- **Actions**: "pick up", "move", "open"

### Intent Parser Implementation

```python
import re
from typing import Dict, List, Optional

class IntentParser:
    def __init__(self):
        self.action_patterns = {
            'navigation': [
                r'go to (.+)',
                r'move to (.+)',
                r'walk to (.+)',
                r'go (.+)',
            ],
            'manipulation': [
                r'pick up (.+)',
                r'grab (.+)',
                r'get (.+)',
                r'take (.+)',
                r'put (.+) in (.+)',
                r'place (.+) on (.+)',
            ],
            'object_interaction': [
                r'open (.+)',
                r'close (.+)',
                r'turn on (.+)',
                r'turn off (.+)',
                r'push (.+)',
                r'pull (.+)',
            ]
        }

    def parse_intent(self, text: str) -> Optional[Dict]:
        """Parse natural language command into structured intent"""
        text = text.lower().strip()

        for action_type, patterns in self.action_patterns.items():
            for pattern in patterns:
                match = re.match(pattern, text)
                if match:
                    groups = match.groups()
                    return {
                        'action_type': action_type,
                        'parameters': list(groups),
                        'original_text': text
                    }

        return None
```

### Handling Ambiguity and Uncertainty

Natural language commands often contain ambiguity that must be resolved:

#### Disambiguation Strategies
- **Contextual Resolution**: Use environmental context to resolve ambiguous references
- **Clarification Requests**: Ask for clarification when multiple interpretations exist
- **Confidence Thresholds**: Only execute commands with high confidence scores

#### Example: Resolving "That Object"
```python
def resolve_deixis(self, command: str, environment_context: dict) -> str:
    """Resolve ambiguous references like 'that object' or 'the red one'"""
    if 'that object' in command:
        # Use object detection to identify the most salient object
        nearest_object = self.get_nearest_salient_object(environment_context)
        return command.replace('that object', nearest_object.name)

    return command
```

## Integrating Language Inputs with ROS 2

### ROS 2 Architecture for Language Commands

The integration of language inputs with ROS 2 involves several components:

#### Language Input Node
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class LanguageInputNode(Node):
    def __init__(self):
        super().__init__('language_input_node')
        self.subscription = self.create_subscription(
            String,
            'voice_commands',
            self.listener_callback,
            10)
        self.command_publisher = self.create_publisher(String, 'parsed_commands', 10)
        self.intent_parser = IntentParser()

    def listener_callback(self, msg):
        """Process incoming voice command"""
        intent = self.intent_parser.parse_intent(msg.data)
        if intent:
            # Publish structured command
            command_msg = String()
            command_msg.data = str(intent)
            self.command_publisher.publish(command_msg)
```

#### Command Execution Node
```python
class CommandExecutionNode(Node):
    def __init__(self):
        super().__init__('command_execution_node')
        self.subscription = self.create_subscription(
            String,
            'parsed_commands',
            self.execute_command,
            10)
        # Additional publishers for robot actions

    def execute_command(self, msg):
        """Execute parsed command on robot"""
        intent = eval(msg.data)  # In practice, use safe deserialization
        action_type = intent['action_type']

        if action_type == 'navigation':
            self.execute_navigation(intent['parameters'])
        elif action_type == 'manipulation':
            self.execute_manipulation(intent['parameters'])
```

### Message Types and Communication Patterns

#### Custom Message Definitions
Create custom message types for language commands:

```python
# language_command.msg
string action_type
string[] parameters
float64 confidence_score
string original_command
```

#### Topic Architecture
- `voice_commands`: Raw transcribed text from speech recognition
- `parsed_intents`: Structured intents from NLP processing
- `robot_actions`: Low-level commands for robot execution
- `confirmation_requests`: Questions to ask the user for clarification

### Example Integration Workflow

```python
class VLAIntegrator:
    def __init__(self):
        self.whisper_interface = RobotWhisperInterface()
        self.intent_parser = IntentParser()
        self.ros2_interface = ROS2Interface()

    def process_voice_command(self, audio_input):
        # Step 1: Transcribe speech to text
        text = self.whisper_interface.transcribe(audio_input)

        # Step 2: Parse intent from text
        intent = self.intent_parser.parse_intent(text)

        # Step 3: Validate and resolve ambiguities
        validated_intent = self.validate_intent(intent)

        # Step 4: Publish to ROS 2
        self.ros2_interface.publish_command(validated_intent)
```

## Handling Ambiguity, Confirmation, and User Feedback

### Ambiguity Detection and Resolution

#### Types of Ambiguity
- **Lexical Ambiguity**: Words with multiple meanings ("bank" - financial institution vs. river bank)
- **Syntactic Ambiguity**: Multiple possible grammatical structures
- **Referential Ambiguity**: Unclear references to objects or locations
- **Scope Ambiguity**: Unclear scope of commands ("Turn off all lights before 10 PM")

#### Resolution Strategies
```python
class AmbiguityResolver:
    def __init__(self):
        self.context_resolver = ContextResolver()
        self.disambiguation_questions = {
            'object_reference': "Do you mean {option1} or {option2}?",
            'location_reference': "Do you mean the {location1} or the {location2}?",
            'action_choice': "Would you like me to {action1} or {action2}?"
        }

    def resolve_ambiguity(self, intent: Dict) -> Dict:
        """Resolve ambiguities in the parsed intent"""
        if self.is_ambiguous(intent):
            clarification_needed = self.generate_clarification(intent)
            user_response = self.request_user_input(clarification_needed)
            return self.update_intent_with_response(intent, user_response)

        return intent
```

### Confirmation Mechanisms

#### Explicit Confirmation
For high-risk or complex commands, request explicit confirmation:

```python
def request_confirmation(self, intent: Dict) -> bool:
    """Request user confirmation before executing command"""
    confirmation_prompt = f"I understood: '{intent['original_text']}'. "
    confirmation_prompt += f"Should I proceed with this action?"

    # Publish to user feedback topic
    self.publish_feedback_request(confirmation_prompt)

    # Wait for user response
    user_response = self.wait_for_response(timeout=30.0)
    return self.parse_affirmative_response(user_response)
```

#### Implicit Confirmation
Provide feedback about the planned action:

```python
def provide_action_feedback(self, intent: Dict):
    """Provide feedback about the planned action"""
    feedback_msg = f"Planning to {intent['action_type']} with parameters: {intent['parameters']}"
    self.publish_system_status(feedback_msg)
```

### Designing Robust Human-Robot Interaction Loops

#### Turn-Taking Protocols
Establish clear protocols for human-robot interaction:

```python
class InteractionManager:
    def __init__(self):
        self.state = 'LISTENING'  # LISTENING, PROCESSING, WAITING_CONFIRMATION, EXECUTING
        self.timeout_config = {
            'listening': 10.0,  # seconds to listen for command
            'processing': 5.0,  # seconds to process command
            'confirmation': 30.0  # seconds to wait for confirmation
        }

    def handle_interaction_turn(self, audio_input=None, user_response=None):
        """Handle interaction turn based on current state"""
        if self.state == 'LISTENING':
            if audio_input:
                self.state = 'PROCESSING'
                return self.process_command(audio_input)

        elif self.state == 'WAITING_CONFIRMATION':
            if user_response:
                self.state = 'EXECUTING'
                return self.execute_with_response(user_response)
```

#### Error Recovery Strategies
Handle various types of errors gracefully:

```python
class ErrorRecoveryHandler:
    def handle_recognition_error(self, error_type: str):
        """Handle speech recognition errors"""
        if error_type == 'NO_SPEECH_DETECTED':
            self.speak("I didn't hear anything. Could you repeat your command?")
        elif error_type == 'LOW_CONFIDENCE':
            self.speak("I'm not sure I understood correctly. Could you repeat that?")
        elif error_type == 'AMBIGUOUS_COMMAND':
            self.handle_ambiguity()

    def handle_execution_error(self, command: Dict, error: Exception):
        """Handle command execution errors"""
        self.log_error(error)
        self.provide_error_feedback(command, error)
        self.offer_alternatives()
```

## Practical Implementation Example

### Complete Voice Command System

Let's implement a complete system that integrates all components:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import whisper
import numpy as np

class VoiceCommandRobot(Node):
    def __init__(self):
        super().__init__('voice_command_robot')

        # Initialize Whisper model
        self.whisper_model = whisper.load_model("base")

        # Subscribe to voice commands
        self.voice_sub = self.create_subscription(
            String, 'raw_voice_input', self.voice_callback, 10)

        # Publish parsed commands
        self.command_pub = self.create_publisher(String, 'parsed_commands', 10)

        # Initialize intent parser
        self.intent_parser = IntentParser()
        self.ambiguity_resolver = AmbiguityResolver()

        # Timer for periodic processing
        self.timer = self.create_timer(0.1, self.process_pending_commands)
        self.pending_audio = None

    def voice_callback(self, msg):
        """Receive raw audio data from microphone"""
        self.pending_audio = msg.data

    def process_pending_commands(self):
        """Process pending audio commands"""
        if self.pending_audio:
            # Transcribe audio
            text = self.transcribe_audio(self.pending_audio)

            # Parse intent
            intent = self.intent_parser.parse_intent(text)

            if intent:
                # Resolve ambiguities
                resolved_intent = self.ambiguity_resolver.resolve_ambiguity(intent)

                # Publish command
                cmd_msg = String()
                cmd_msg.data = str(resolved_intent)
                self.command_pub.publish(cmd_msg)

            self.pending_audio = None

    def transcribe_audio(self, audio_data):
        """Transcribe audio data using Whisper"""
        # Convert audio data to format expected by Whisper
        audio_array = np.frombuffer(audio_data, dtype=np.float32)
        result = self.whisper_model.transcribe(audio_array, fp16=False)
        return result['text']

def main(args=None):
    rclpy.init(args=args)
    robot = VoiceCommandRobot()

    try:
        rclpy.spin(robot)
    except KeyboardInterrupt:
        pass
    finally:
        robot.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Connecting to Previous Modules

This chapter builds upon concepts from previous modules:
- **Module 1 (ROS 2)**: Uses ROS 2 communication patterns for language command integration
- **Module 2 (Digital Twin)**: Simulation environments can be used to test voice command systems
- **Module 3 (NVIDIA Isaac)**: Perception capabilities support object recognition for ambiguous commands

## Summary

This chapter covered the implementation of voice-to-action interfaces using OpenAI Whisper and ROS 2 integration. We explored speech-to-text conversion, intent recognition, ambiguity handling, and robust human-robot interaction loops. The practical implementation examples demonstrate how to create a complete voice command system for robotic applications.

The next chapter will focus on the capstone project, integrating all components into a complete autonomous humanoid system.

## Exercises

1. Implement a simple Whisper-based speech recognition system that can distinguish between navigation and manipulation commands.
2. Create an intent parser that can handle complex commands with multiple objects and spatial relationships.
3. Design a confirmation mechanism for high-risk commands that ensures user safety.