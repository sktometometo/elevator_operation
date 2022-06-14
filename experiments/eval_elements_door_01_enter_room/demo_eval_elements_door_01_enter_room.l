#!/usr/bin/env roseus

(ros::roseus-add-srvs "elevator_operation")
(ros::roseus-add-msgs "move_base_msgs")
(ros::roseus-add-msgs "std_msgs")

(load "package://fetcheus/fetch-interface.l")
(load "package://elevator_operation/euslisp/utils.l")

(setq *door-is-open* nil)
(defun callback-door-is-open (msg)
  (setq *door-is-open* (send msg :data))
  )

(defun main()
  (fetch-init)
  (ros::subscribe "~door_is_open" std_msgs::Bool callback-door-is-open)

  (setq *door-frame-id* (ros::get-param "~door_frame_id" ))
  (setq *pose-inside-room* (ros::get-param "~pose_inside_room"))

  (call-look-at-target "~look_at" *door-frame-id*)

  (send *ri* :speak-jp "ごめんください")

  (while (ros::ok)
    (ros::spin-once)
    (if *door-is-open*
      (break))
    )

  (send *ri* :speak-jp "失礼します")
  (send-move-base-goal
    (elt *pose-inside-room* 0)
    (elt *pose-inside-room* 1)
    (elt *pose-inside-room* 2)
    (elt *pose-inside-room* 3)
    (elt *pose-inside-room* 4)
    (elt *pose-inside-room* 5)
    (elt *pose-inside-room* 6)
    (elt *pose-inside-room* 7)
    :wait t
    )

  (ros::ros-info "Finished.")
  )
