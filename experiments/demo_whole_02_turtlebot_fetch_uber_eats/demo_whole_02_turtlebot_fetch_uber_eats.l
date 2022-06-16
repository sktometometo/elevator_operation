#!/usr/bin/env roseus

(ros::roseus-add-msgs "move_base_msgs")
(ros::roseus-add-msgs "elevator_operation")
(ros::roseus-add-msgs "geometry_msgs")

(load "package://dxl_armed_turtlebot/euslisp/dxl-armed-turtlebot-interface.l")

(dxl-armed-turtlebot-init)

(setq *move-base-action-client* (instance ros::simple-action-client :init "/move_base" move_base_msgs::MoveBaseAction))
(send *move-base-action-client* :wait-for-server)

(setq *move-elevator-action-client* (instance ros::simple-action-client :init "/move_elevator" move_elevator::MoveElevatorAction))
(send *move-elevator-action-client* :wait-for-server)

(defun move-to (x y &optional (theta_deg 0) &key (frame-id "base_link") (wait t))
  (let ((theta (deg2rad theta_deg))
        (goal (instance move_base_msgs::MoveBaseGoal :init)))
    (send goal :target_pose :header :frame_id frame-id)
    (send goal :target_pose :pose :position :x x)
    (send goal :target_pose :pose :position :y y)
    (send goal :target_pose :pose :position :z 0)
    (send goal :target_pose :pose :orientation :x 0)
    (send goal :target_pose :pose :orientation :y 0)
    (send goal :target_pose :pose :orientation :z (sin (/ theta 2)))
    (send goal :target_pose :pose :orientation :w (cos (/ theta 2)))
    (send *move-base-action-client* :send-goal goal)
    (if wait
        (send *move-base-action-client* :wait-for-result)
        )
    ))

(defun move-elevator (target-floor-name)
  (let (result (goal (instance move_elevator::MoveElevatorGoal :init)))
    (send goal :target_floor target-floor)
    (send *move-elevator-action-client* :send-goal goal)
    (send *move-elevator-action-client* :wait-for-result)
    (setq result (send *move-elevator-action-client* :get-result))
    (send result :success)))

(defun get-current-floor ()
  )

(defun wait-until-button-pressed ()
  ;; TODO
  t
  )

(defun get-pick-up-place ()
  (let (pose-stamped)
    (setq pose-stamped (instance geometry_msgs::PoseStamped :init))
    (send pose-stamped :header :frame_id "eng2/1f")
    (send pose-stamped :pose :pose :position :x -9.0)
    (send pose-stamped :pose :pose :position :y 2.2)
    (send pose-stamped :pose :pose :position :z 0.0)
    (send pose-stamped :pose :pose :orientation :x 0)
    (send pose-stamped :pose :pose :orientation :y 0)
    (send pose-stamped :pose :pose :orientation :z 0)
    (send pose-stamped :pose :pose :orientation :w 1.0)
    (list 1 pose-stamped)))

(defun get-initial-place ()
  (let (pose-stamped)
    (setq pose-stamped (instance geometry_msgs::PoseStamped :init))
    (send pose-stamped :header :frame_id "eng2/7f")
    (send pose-stamped :pose :pose :position :x 2.1)
    (send pose-stamped :pose :pose :position :y 6.5)
    (send pose-stamped :pose :pose :position :z 0.0)
    (send pose-stamped :pose :pose :orientation :x 0)
    (send pose-stamped :pose :pose :orientation :y 0)
    (send pose-stamped :pose :pose :orientation :z 0)
    (send pose-stamped :pose :pose :orientation :w 1.0)
    (list 7 pose-stamped)))

(defun main ()
  (get
  )
