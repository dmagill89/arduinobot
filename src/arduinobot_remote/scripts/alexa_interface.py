#!/usr/bin/env python3
from flask import Flask
from ask_sdk_core.skill_builder import SkillBuilder
from flask_ask_sdk.skill_adapter import SkillAdapter
from ask_sdk_core.dispatch_components import AbstractRequestHandler
from ask_sdk_core.utils import is_request_type, is_intent_name
from ask_sdk_core.handler_input import HandlerInput
from ask_sdk_model import Response
from ask_sdk_model.ui import SimpleCard
from ask_sdk_core.dispatch_components import AbstractExceptionHandler
import rospy
import actionlib
import threading
from arduinobot_remote.msg import ArduinobotTaskAction, ArduinobotTaskGoal

threading.Thread(target=lambda: rospy.init_node("alexa_interface", disable_signals=True)).start()
client = actionlib.SimpleActionClient("task_server", ArduinobotTaskAction)

app = Flask(__name__)

class LaunchRequestHandler(AbstractRequestHandler):
    def can_handle(self, handler_input):
        # type: (HandlerInput) -> bool
        return is_request_type("LaunchRequest")(handler_input)

    def handle(self, handler_input):
        # type: (HandlerInput) -> Response
        speech_text = "Hi, how can I help?"

        handler_input.response_builder.speak(speech_text).set_card(
            SimpleCard("Hello World", speech_text)).set_should_end_session(
            False)
        
        goal = ArduinobotTaskGoal(task_number=0)
        client.send_goal(goal)

        return handler_input.response_builder.response


class PickIntentHandler(AbstractRequestHandler):
    def can_handle(self, handler_input):
        # type: (HandlerInput) -> bool
        return is_intent_name("PickIntent")(handler_input)

    def handle(self, handler_input):
        # type: (HandlerInput) -> Response
        speech_text = "Okay, I'm moving"

        handler_input.response_builder.speak(speech_text).set_card(
            SimpleCard("Pick", speech_text)).set_should_end_session(
            True)
        
        goal = ArduinobotTaskGoal(task_number=1)
        client.send_goal(goal)

        return handler_input.response_builder.response
    

class SleepIntentHandler(AbstractRequestHandler):
    def can_handle(self, handler_input):
        # type: (HandlerInput) -> bool
        return is_intent_name("SleepIntent")(handler_input)

    def handle(self, handler_input):
        # type: (HandlerInput) -> Response
        speech_text = "Okay, see you later."

        handler_input.response_builder.speak(speech_text).set_card(
            SimpleCard("Sleep", speech_text)).set_should_end_session(
            True)
        
        goal = ArduinobotTaskGoal(task_number=2)
        client.send_goal(goal)

        return handler_input.response_builder.response


class WakeIntentHandler(AbstractRequestHandler):
    def can_handle(self, handler_input):
        # type: (HandlerInput) -> bool
        return is_intent_name("WakeIntent")(handler_input)

    def handle(self, handler_input):
        # type: (HandlerInput) -> Response
        speech_text = "Hi, I am ready."

        handler_input.response_builder.speak(speech_text).set_card(
            SimpleCard("Wake", speech_text)).set_should_end_session(
            True)
        
        goal = ArduinobotTaskGoal(task_number=0)
        client.send_goal(goal)
        return handler_input.response_builder.response    

class RightIntentHandler(AbstractRequestHandler):
    def can_handle(self, handler_input):
        # type: (HandlerInput) -> bool
        return is_intent_name("RightIntent")(handler_input)

    def handle(self, handler_input):
        # type: (HandlerInput) -> Response
        speech_text = "Okay, I'm moving right"

        handler_input.response_builder.speak(speech_text).set_card(
            SimpleCard("Right", speech_text)).set_should_end_session(
            True)
        
        goal = ArduinobotTaskGoal(task_number=3)
        client.send_goal(goal)
        return handler_input.response_builder.response
    

class LeftIntentHandler(AbstractRequestHandler):
    def can_handle(self, handler_input):
        # type: (HandlerInput) -> bool
        return is_intent_name("LeftIntent")(handler_input)

    def handle(self, handler_input):
        # type: (HandlerInput) -> Response
        speech_text = "Okay, I'm moving left"

        handler_input.response_builder.speak(speech_text).set_card(
            SimpleCard("Left", speech_text)).set_should_end_session(
            True)
        
        goal = ArduinobotTaskGoal(task_number=4)
        client.send_goal(goal)
        return handler_input.response_builder.response
    

class DownIntentHandler(AbstractRequestHandler):
    def can_handle(self, handler_input):
        # type: (HandlerInput) -> bool
        return is_intent_name("DownIntent")(handler_input)

    def handle(self, handler_input):
        # type: (HandlerInput) -> Response
        speech_text = "Okay, I'm moving down."

        handler_input.response_builder.speak(speech_text).set_card(
            SimpleCard("Down", speech_text)).set_should_end_session(
            True)
        
        goal = ArduinobotTaskGoal(task_number=5)
        client.send_goal(goal)
        return handler_input.response_builder.response


class UpIntentHandler(AbstractRequestHandler):
    def can_handle(self, handler_input):
        # type: (HandlerInput) -> bool
        return is_intent_name("UpIntent")(handler_input)

    def handle(self, handler_input):
        # type: (HandlerInput) -> Response
        speech_text = "Okay, I'm moving up."

        handler_input.response_builder.speak(speech_text).set_card(
            SimpleCard("Up", speech_text)).set_should_end_session(
            True)
        
        goal = ArduinobotTaskGoal(task_number=6)
        client.send_goal(goal)
        return handler_input.response_builder.response
    
class CloseIntentHandler(AbstractRequestHandler):
    def can_handle(self, handler_input):
        # type: (HandlerInput) -> bool
        return is_intent_name("CloseIntent")(handler_input)

    def handle(self, handler_input):
        # type: (HandlerInput) -> Response
        speech_text = "Okay, I'm closing the gripper"

        handler_input.response_builder.speak(speech_text).set_card(
            SimpleCard("Up", speech_text)).set_should_end_session(
            True)
        
        goal = ArduinobotTaskGoal(task_number=7)
        client.send_goal(goal)
        return handler_input.response_builder.response
    
    
class OpenIntentHandler(AbstractRequestHandler):
    def can_handle(self, handler_input):
        # type: (HandlerInput) -> bool
        return is_intent_name("OpenIntent")(handler_input)

    def handle(self, handler_input):
        # type: (HandlerInput) -> Response
        speech_text = "Okay, I'm opening the gripper"

        handler_input.response_builder.speak(speech_text).set_card(
            SimpleCard("Up", speech_text)).set_should_end_session(
            True)
        
        goal = ArduinobotTaskGoal(task_number=8)
        client.send_goal(goal)
        return handler_input.response_builder.response


class AllExceptionHandler(AbstractExceptionHandler):

    def can_handle(self, handler_input, exception):
        # type: (HandlerInput, Exception) -> bool
        return True

    def handle(self, handler_input, exception):
        # type: (HandlerInput, Exception) -> Response
        # Log the exception in CloudWatch Logs
        print(exception)

        speech = "Sorry, I didn't get that. Can you repeat please?"
        handler_input.response_builder.speak(speech).ask(speech)
        return handler_input.response_builder.response


skill_builder = SkillBuilder()
skill_builder.add_request_handler(LaunchRequestHandler())
skill_builder.add_request_handler(PickIntentHandler())
skill_builder.add_request_handler(SleepIntentHandler())
skill_builder.add_request_handler(WakeIntentHandler())
skill_builder.add_exception_handler(AllExceptionHandler())
# Register your intent handlers to the skill_builder object


skill_adapter = SkillAdapter(
    skill=skill_builder.create(), skill_id="amzn1.ask.skill.67704ad5-a04b-4cca-8ae4-501ef9a0178d", app=app)


skill_adapter.register(app=app, route="/")


if __name__ == '__main__':
    app.run()
