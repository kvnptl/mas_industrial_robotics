#!/usr/bin/env python

import mcr_states.common.basic_states as gbs
import mir_states.common.manipulation_states as gms  # move the arm, and gripper
import rospy
import smach
from mir_actions.utils import Utils
from mir_planning_msgs.msg import (
    GenericExecuteAction,
    GenericExecuteFeedback,
    GenericExecuteResult,
    GenericExecuteGoal
)
from smach_ros import ActionServerWrapper, IntrospectionServer
from std_msgs.msg import String
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import PoseStamped
from diagnostic_msgs.msg import KeyValue
from actionlib import SimpleActionClient
from actionlib_msgs.msg import GoalStatus


# ===============================================================================
class GetPoseToPlaceOject(smach.State):  # inherit from the State base class
    def __init__(self, topic_name_pub, topic_name_sub, event_sub, timeout_duration):
        smach.State.__init__(
            self,
            outcomes=["succeeded", "failed"],
            input_keys=["goal", "feedback"],
            output_keys=["feedback", "result", "move_arm_to"],
        )

        self.timeout = rospy.Duration.from_sec(timeout_duration)
        # create publisher
        self.platform_name_pub = rospy.Publisher(topic_name_pub, String, queue_size=10)
        rospy.Subscriber(topic_name_sub, String, self.pose_cb)
        rospy.Subscriber(event_sub, String, self.event_cb)

        rospy.sleep(0.1)  # time for publisher to register

        self.place_pose = None
        self.status = None
	self.empty_location = None

    def pose_cb(self, msg):
        self.place_pose = msg.data

    def event_cb(self, msg):
        self.status = msg.data

    def execute(self, userdata):
        # Add empty result msg (because if none of the state do it, action server gives error)
        userdata.result = GenericExecuteResult()
        userdata.feedback = GenericExecuteFeedback(
            current_state="GetPoseToPlaceOject", text="Getting pose to place obj",
        )

        location = Utils.get_value_of(userdata.goal.parameters, "location")

        if location is None:
            rospy.logwarn('"location" not provided. Using default.')
            return "failed"

        self.place_pose = None
        self.status = None
        self.platform_name_pub.publish(String(data=location))

        # wait for messages to arrive
        start_time = rospy.Time.now()
        rate = rospy.Rate(10)  # 10hz
        while not (rospy.is_shutdown()):
            if rospy.Time.now() - start_time > self.timeout:
                break
            if self.place_pose is not None and self.status is not None:
                break
            rate.sleep()

        if (
            self.place_pose is not None
            and self.status is not None
            and self.status == "e_success"
        ):
            userdata.move_arm_to = self.place_pose
            return "succeeded"
        else:
            return "failed"


# ===============================================================================


class CheckIfLocationIsShelf(smach.State):
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=["shelf", "not_shelf"],
            input_keys=["goal"],
            output_keys=["feedback", "result"],
        )

    def execute(self, userdata):
        location = Utils.get_value_of(userdata.goal.parameters, "location")
        print("[Place Object Server] Location received : ", location)
        if (location == "SH01") or (location == "SH02"):
            return "shelf"
        else:
            return "not_shelf"

# ==============================================================================

# new class for empty space detection

class GetEmptyPositionOnTable(smach.State):
    def __init__(self,empty_spaces,topic_name_pub):
	smach.State.__init__(
		self,
		outcomes=["success", "failure"],
		input_keys=["empty_locations",],
		output_keys=["feedback","result","empty_locations"])
	self.empty_locations = None

        self.timeout = rospy.Duration.from_sec(15.0)
        rospy.Subscriber(empty_spaces, PoseArray, self.empty_space_cb) # subscribing to empty_space locations published by empty_space_detector node
        rospy.sleep(0.1)

    def empty_space_cb(self, msg): # Callback for get empty_space_location from empty_space detector node 
        
	self.empty_locations = msg

    def execute(self, userdata): # Getting data from empty space detector and giving all the poses it to next states 
        
	userdata.result = GenericExecuteResult()
        userdata.feedback = GenericExecuteFeedback(
            current_state="POSE_RECEIVE", text="Receiving empty space location",)

	userdata.empty_locations = self.empty_locations # The empty locations have all three empty poses 

	return 'success'

# ===============================================================================

# new class for publishing the empty pose

class PublishObjectPose(smach.State):
    def __init__(self, empty_pose_index):
        smach.State.__init__(self, outcomes=["success", "failed"],
                                    input_keys=["empty_locations"])

        self.empty_pose_pub = rospy.Publisher(
            "/mcr_perception/object_selector/output/object_pose",
            PoseStamped,
            queue_size=10)
	self.selection_index = empty_pose_index

    def execute(self, userdata):

	empty_locations = userdata.empty_locations # Passing the empty location and publising the poses

	# converiting the pose into desired format
        single_array = PoseStamped()
        single_array.header = empty_locations.header
        single_array.pose = empty_locations.poses[self.selection_index]

        rospy.loginfo("Publishing single pose to pregrasp planner")

        rospy.loginfo(type(single_array))
        self.empty_pose_pub.publish(single_array)

	rospy.sleep(0.3)
	single_array = None
        return "success"


# class from unstage server for unstaging the object 

class Unstage_to_place(smach.State):

    def __init__(self):

        smach.State.__init__(self, outcomes=["success","failed"],input_keys=["goal","heavy_objects", "platform","object"])
	self.platform = "PLATFORM_MIDDLE"
	self.obj = "M20"

    def execute(self,userdata):

	self.platform = Utils.get_value_of(userdata.goal.parameters, "platform")
	self.obj = Utils.get_value_of(userdata.goal.parameters, "object")

	if self.obj is None:
            rospy.logwarn('Missing parameter "object". Using default.')
            self.obj = "light"
        for heavy_object in userdata.heavy_objects:
            if heavy_object.upper() in self.obj.upper():
                self.obj =  "heavy"
	    else:
        	self.obj =  "light"


        self.unstage_client = SimpleActionClient('unstage_object_server', GenericExecuteAction)
        self.unstage_client.wait_for_server()

	# Assigning the goal

	goal = GenericExecuteGoal()
        goal.parameters.append(KeyValue(key="platform", value=self.platform))
        goal.parameters.append(KeyValue(key="object", value=self.obj))

        self.unstage_client.send_goal(goal)

        self.unstage_client.wait_for_result(rospy.Duration.from_sec(15.0))
        rospy.loginfo("Unstaged from backplatform " + self.platform)
        rospy.loginfo("Sending following goal to unstage object server")
        rospy.loginfo(goal)

	return "success"

#=================================================================================

def transition_cb(*args, **kwargs):
    userdata = args[0]
    sm_state = args[1][0]

    feedback = GenericExecuteFeedback()
    feedback.current_state = sm_state
    userdata.feedback = feedback

def start_cb(*args, **kwargs):
    userdata = args[0]
    sm_state = args[1][0]

    feedback = GenericExecuteFeedback()
    feedback.current_state = sm_state
    userdata.feedback = feedback


def main():
    rospy.init_node("place_object_server")
    # Construct state machine
    sm = smach.StateMachine(
        outcomes=["OVERALL_SUCCESS", "OVERALL_FAILED"],
        input_keys=["goal", "feedback", "result"],
        output_keys=["feedback", "result"],)


    sm.userdata.empty_locations = None
    sm.userdata.heavy_objects = rospy.get_param("~heavy_objects", ["m20_100"])
    with sm:
        # add states to the container
        smach.StateMachine.add(
            "CHECK_IF_SHELF_INITIAL",
            CheckIfLocationIsShelf(),
            transitions={
                "shelf": "MOVE_ARM_TO_SHELF_INTERMEDIATE",
                "not_shelf": "MOVE_ARM_TO_PRE_PLACE",
            },
        )

        smach.StateMachine.add(
            "MOVE_ARM_TO_SHELF_INTERMEDIATE",
            gms.move_arm("shelf_intermediate"),
            transitions={
                "succeeded": "MOVE_ARM_TO_SHELF_INTERMEDIATE_2",
                "failed": "MOVE_ARM_TO_SHELF_INTERMEDIATE",
            },
        )

        smach.StateMachine.add(
            "MOVE_ARM_TO_SHELF_INTERMEDIATE_2",
            gms.move_arm("shelf_intermediate_2"),
            transitions={
                "succeeded": "MOVE_ARM_TO_PRE_GRASP_LOWER",
                "failed": "MOVE_ARM_TO_SHELF_INTERMEDIATE_2",
            },
        )

        smach.StateMachine.add(
            "MOVE_ARM_TO_PRE_GRASP_LOWER",
            gms.move_arm("shelf_pre_grasp_lower"),
            transitions={
                "succeeded": "MOVE_ARM_TO_SHELF_PLACE_FINAL",
                "failed": "MOVE_ARM_TO_PRE_GRASP_LOWER",
            },
        )

        smach.StateMachine.add(
            "MOVE_ARM_TO_SHELF_PLACE_FINAL",
            gms.move_arm("shelf_place_final"),
            transitions={
                "succeeded": "OPEN_GRIPPER_SHELF",
                "failed": "MOVE_ARM_TO_SHELF_PLACE_FINAL",
            },
        )

        smach.StateMachine.add(
            "OPEN_GRIPPER_SHELF",
            gms.control_gripper("open"),
            transitions={"succeeded": "MOVE_ARM_TO_SHELF_PLACE_FINAL_RETRACT"},
        )

        smach.StateMachine.add(
            "MOVE_ARM_TO_SHELF_PLACE_FINAL_RETRACT",
            gms.move_arm("shelf_place_final"),
            transitions={
                "succeeded": "MOVE_ARM_TO_PRE_GRASP_LOWER_RETRACT",
                "failed": "MOVE_ARM_TO_SHELF_PLACE_FINAL_RETRACT",
            },
        )

        smach.StateMachine.add(
            "MOVE_ARM_TO_PRE_GRASP_LOWER_RETRACT",
            gms.move_arm("shelf_pre_grasp_lower"),
            transitions={
                "succeeded": "MOVE_ARM_TO_SHELF_INTERMEDIATE_2_RETRACT",
                "failed": "MOVE_ARM_TO_PRE_GRASP_LOWER_RETRACT",
            },
        )

        smach.StateMachine.add(
            "MOVE_ARM_TO_SHELF_INTERMEDIATE_2_RETRACT",
            gms.move_arm("shelf_intermediate_2"),
            transitions={
                "succeeded": "MOVE_ARM_TO_SHELF_INTERMEDIATE_RETRACT",
                "failed": "MOVE_ARM_TO_SHELF_INTERMEDIATE_2_RETRACT",
            },
        )

        smach.StateMachine.add(
            "MOVE_ARM_TO_SHELF_INTERMEDIATE_RETRACT",
            gms.move_arm("shelf_intermediate"),
            transitions={
                "succeeded": "MOVE_ARM_TO_NEUTRAL",
                "failed": "MOVE_ARM_TO_SHELF_INTERMEDIATE_RETRACT",
            },
        )


        smach.StateMachine.add(
            "MOVE_ARM_TO_PRE_PLACE",
            gms.move_arm("look_at_workspace"), # New change from turntable to workspace
            transitions={
                "succeeded": "EMPTY_POSITION_SELECTION",
                "failed": "MOVE_ARM_TO_PRE_PLACE",
            },
        )


	smach.StateMachine.add("EMPTY_POSITION_SELECTION",

		gbs.send_and_wait_events_combined(
			event_in_list = [("/mir_perception/empty_space_detector/event_in","e_add_cloud")],
			event_out_list = [("/mir_perception/empty_space_detector/event_out","e_added_cloud", True)],
			timeout_duration=50,),
	transitions={"success": "TRIGGER",
		    "timeout": "EMPTY_POSITION_SELECTION",
		    "failure": "EMPTY_POSITION_SELECTION",},
	)


        smach.StateMachine.add("TRIGGER",

                gbs.send_and_wait_events_combined(
                   event_in_list = [("/mir_perception/empty_space_detector/event_in","e_trigger")],
		   event_out_list = [("/mir_perception/empty_space_detector/event_out","e_success",True)],
		   timeout_duration = 50,),
        transitions={"success": "POSE_RECEIVE",
		     "timeout": "TRIGGER",
		     "failure": "TRIGGER",},
        )



        smach.StateMachine.add(
		"POSE_RECEIVE",
                GetEmptyPositionOnTable(
			"/mir_perception/empty_space_detector/empty_spaces",
			"/mcr_perception/place_pose_selector/empty_space_pose_array"),

        transitions={
                "success": "PUBLISH_OBJECT_POSE_1",
                "failure": "MOVE_ARM_TO_NEUTRAL",
	   },
	)


        smach.StateMachine.add(
            "PUBLISH_OBJECT_POSE_1",
            PublishObjectPose(0),
            transitions={"success": "CHECK_PRE_GRASP_POSE", "failed": "PUBLISH_OBJECT_POSE_2"}

        )

        smach.StateMachine.add(
            "PUBLISH_OBJECT_POSE_2",
            PublishObjectPose(1),
            transitions={"success": "CHECK_PRE_GRASP_POSE", "failed": "PUBLISH_OBJECT_POSE_3"}

        )

        smach.StateMachine.add(
            "PUBLISH_OBJECT_POSE_3",
            PublishObjectPose(2),
            transitions={"success": "CHECK_PRE_GRASP_POSE", "failed": "OVERALL_FAILED"}

        )
        smach.StateMachine.add(
            "CHECK_PRE_GRASP_POSE",
            gbs.send_and_wait_events_combined(
                event_in_list=[
                    ("/pregrasp_planner_node/event_in", "e_start")
                ],
                event_out_list=[
                    (
                        "/pregrasp_planner_node/event_out",
                        "e_success",
                        True,
                    )
                ],
                timeout_duration=20,
            ),
            transitions={
                "success": "UNSTAGE_FOR_PLACING",
                "timeout": "OVERALL_FAILED",
                "failure": "EMPTY_POSITION_SELECTION",
            },
        )



        smach.StateMachine.add(
                "UNSTAGE_FOR_PLACING",
                Unstage_to_place(),
        transitions={
                "success":"GO_TO_PRE_GRASP_POSE", # change it to OPEN_GRIPPER when gripper is repaired 
                "failed":"MOVE_ARM_TO_NEUTRAL",
          },
        )


        smach.StateMachine.add(
            "GO_TO_PRE_GRASP_POSE",
            gbs.send_and_wait_events_combined(
                event_in_list=[
                    ("/waypoint_trajectory_generation/event_in", "e_start")
                ],
                event_out_list=[
                    (
                        "/waypoint_trajectory_generation/event_out",
                        "e_success",
                        True,
                    )
                ],
                timeout_duration=20,
            ),
            transitions={
                "success": "MOVE_ARM_TO_NEUTRAL",  # change it to OPEN_GRIPPER after repairing the gripper
                "timeout": "OVERALL_FAILED",
                "failure": "OVERALL_FAILED",
            },
        )


        smach.StateMachine.add(
                    "OPEN_GRIPPER",
                    gms.control_gripper("open"),
                    transitions={
			"succeeded": "MOVE_ARM_TO_NEUTRAL"},
                )


        smach.StateMachine.add(
                    "MOVE_ARM_TO_NEUTRAL",
                    gms.move_arm("barrier_tape"),
                    transitions={
                        "succeeded": "OVERALL_SUCCESS",
                        "failed": "MOVE_ARM_TO_NEUTRAL",
                    },
                )

    sm.register_transition_cb(transition_cb)
    sm.register_start_cb(start_cb)

    # smach viewer
    if rospy.get_param("~viewer_enabled", True):
        sis = IntrospectionServer(
            "place_object_smach_viewer", sm, "/STAGE_OBJECT_SMACH_VIEWER"
        )
        sis.start()

    # Construct action server wrapper
    asw = ActionServerWrapper(
        server_name="place_object_server",
        action_spec=GenericExecuteAction,
        wrapped_container=sm,
        succeeded_outcomes=["OVERALL_SUCCESS"],
        aborted_outcomes=["OVERALL_FAILED"],
        preempted_outcomes=["PREEMPTED"],
        goal_key="goal",
        feedback_key="feedback",
        result_key="result",
    )
    # Run the server in a background thread
    asw.run_server()
    rospy.spin()


if __name__ == "__main__":
    main()
