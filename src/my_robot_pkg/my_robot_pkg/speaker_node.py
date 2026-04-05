import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading
import os
import json
from google.cloud import speech, texttospeech
from google import genai
from google.genai import types
import pyaudio
import wave
import io
import speech_recognition as sr  # For easier microphone handling

class SpeakerNode(Node):
    def __init__(self):
        super().__init__('speaker_node')

        # Publishers
        self.speaker_control_pub = self.create_publisher(String, 'speaker_control', 10)

        # Subscribers
        self.create_subscription(String, 'speaker_control', self.speaker_control_callback, 10)
        self.create_subscription(String, 'maincontrolling', self.maincontrolling_callback, 10)

        # Google API setup
        # Assume credentials file is in the package directory
        credentials_path = os.path.join(os.path.dirname(__file__), 'google_credentials.json')
        os.environ['GOOGLE_APPLICATION_CREDENTIALS'] = credentials_path

        # Gemini API client
        api_key = os.environ.get('GEMINI_API_KEY') or 'AIzaSyDdVHDj3zLaR-JEA3Xs7Hl3jFaZWaBG_lg'
        self.genai_client = genai.Client(api_key=api_key)
        self.model_name = 'gemini-2-flash-preview'

        # Audio settings
        self.audio_format = speech.RecognitionConfig.AudioEncoding.LINEAR16
        self.sample_rate = 16000
        self.channels = 1

        self.get_logger().info('SpeakerNode initialized.')

    def speaker_control_callback(self, msg):
        data = msg.data.strip()
        self.get_logger().info(f'Received speaker_control: {data}')

        if data == 'requestopen':
            self.handle_request_open()
        elif data == 'requestclose':
            self.handle_request_close()
        elif data == 'emergency':
            self.handle_emergency()
        # Add more cases as needed

    def maincontrolling_callback(self, msg):
        data = msg.data.strip()
        self.get_logger().info(f'Received maincontrolling: {data}')

        if data == 'speakeractive':
            self.handle_speaker_active()
        # Add more cases as needed

    def handle_request_open(self):
        # Announce arrival and opening
        announcement = "Wheelchair has arrived. Be careful, chair is opening. Can I proceed?"
        self.speak_text(announcement)

        # Listen for confirmation
        self.get_logger().info('Listening for user confirmation...')
        response = self.listen_for_speech()
        if response and ('yes' in response.lower() or 'proceed' in response.lower()):
            self.get_logger().info('User confirmed: granting speaker permission.')
            self.publish_granted_speaker()
        else:
            self.get_logger().info('User did not confirm or no response.')

    def handle_request_close(self):
        # Announce closing and request confirmation
        announcement = "Chair is closing. Can I proceed?"
        self.speak_text(announcement)

        # Listen for confirmation
        self.get_logger().info('Listening for user confirmation...')
        response = self.listen_for_speech()
        if response and ('yes' in response.lower() or 'proceed' in response.lower()):
            self.get_logger().info('User confirmed: granting speaker permission for close.')
            self.publish_granted_speaker()
        else:
            self.get_logger().info('User did not confirm or no response for close.')

    def handle_emergency(self):
        self.get_logger().info('Emergency triggered, playing elevenlabs sound track.')
        audio_file_path = os.path.join(os.path.dirname(__file__), 'elevenlabs.wav')  # Assuming WAV file
        self.play_audio_file(audio_file_path)

    def play_audio_file(self, file_path):
        try:
            with wave.open(file_path, 'rb') as wf:
                p = pyaudio.PyAudio()
                stream = p.open(format=p.get_format_from_width(wf.getsampwidth()),
                                channels=wf.getnchannels(),
                                rate=wf.getframerate(),
                                output=True)
                data = wf.readframes(1024)
                while data:
                    stream.write(data)
                    data = wf.readframes(1024)
                stream.stop_stream()
                stream.close()
                p.terminate()
        except Exception as e:
            self.get_logger().error(f'Error playing audio file {file_path}: {e}')

    def handle_speaker_active(self):
        self.get_logger().info('Entering AI interaction mode.')
        while True:  # Loop until some exit condition, for now infinite
            self.get_logger().info('Listening for user query...')
            user_text = self.listen_for_speech()
            if user_text:
                self.get_logger().info(f'User said: {user_text}')
                # Send to Gemini
                response_text = self.query_gemini(user_text)
                self.get_logger().info(f'Gemini response: {response_text}')
                # Speak response
                self.speak_text(response_text)
            else:
                self.get_logger().info('No speech detected, continuing...')

    def listen_for_speech(self):
        recognizer = sr.Recognizer()
        with sr.Microphone() as source:
            try:
                audio = recognizer.listen(source, timeout=5, phrase_time_limit=10)
                text = recognizer.recognize_google(audio)
                return text
            except sr.WaitTimeoutError:
                self.get_logger().info('No speech detected within timeout.')
                return None
            except sr.UnknownValueError:
                self.get_logger().info('Could not understand audio.')
                return None
            except sr.RequestError as e:
                self.get_logger().error(f'Speech recognition error: {e}')
                return None

    def speak_text(self, text):
        synthesis_input = texttospeech.SynthesisInput(text=text)
        voice = texttospeech.VoiceSelectionParams(
            language_code='en-US',
            ssml_gender=texttospeech.SsmlVoiceGender.NEUTRAL
        )
        audio_config = texttospeech.AudioConfig(
            audio_encoding=texttospeech.AudioEncoding.LINEAR16
        )
        response = self.tts_client.synthesize_speech(
            input=synthesis_input, voice=voice, audio_config=audio_config
        )

        # Play the audio
        audio_stream = io.BytesIO(response.audio_content)
        with wave.open(audio_stream, 'rb') as wf:
            p = pyaudio.PyAudio()
            stream = p.open(format=p.get_format_from_width(wf.getsampwidth()),
                            channels=wf.getnchannels(),
                            rate=wf.getframerate(),
                            output=True)
            data = wf.readframes(1024)
            while data:
                stream.write(data)
                data = wf.readframes(1024)
            stream.stop_stream()
            stream.close()
            p.terminate()

    def query_gemini(self, prompt):
        try:
            contents = [
                types.Content(
                    role="user",
                    parts=[
                        types.Part.from_text(text=prompt),
                    ],
                ),
            ]
            generate_content_config = types.GenerateContentConfig(
                thinking_config=types.ThinkingConfig(
                    thinking_level="HIGH",
                ),
            )

            response_text = ""
            for chunk in self.genai_client.models.generate_content_stream(
                model=self.model_name,
                contents=contents,
                config=generate_content_config,
            ):
                if text := chunk.text:
                    response_text += text
            
            return response_text if response_text else "No response received."
        except Exception as e:
            self.get_logger().error(f'Gemini API error: {e}')
            return "Sorry, I couldn't process that request."

    def publish_granted_speaker(self):
        msg = String()
        msg.data = 'granted_speaker'
        self.speaker_control_pub.publish(msg)
        self.get_logger().info('Published granted_speaker to speaker_control.')


def main(args=None):
    rclpy.init(args=args)
    node = SpeakerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('SpeakerNode shutting down.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
