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

(setq *rest-elevator* nil)
(defun callback-rest-elevator (msg)
  (setq *rest-elevator* (send msg :data))
  )

(defun add-wall-collision-object (msg)
  )

(defun main()
  (fetch-init)
  (ros::subscribe "~door_is_open" std_msgs::Bool #'callback-door-is-open)
  (ros::subscribe "~rest_elevator" std_msgs::Bool #'callback-rest-elevator)

  (setq *door-frame-id* (ros::get-param "~door_frame_id" ))
  (setq *duration-open* (ros::get-param "~duration_open"))
  (setq *pose-inside-elevator* (ros::get-param "~pose_inside_elevator"))

  (add-collision-objects)

  (move-arm-pre-pose)

  (call-look-at-target "~look_at" *door-frame-id*)

  (while (ros::ok)
    (ros::spin-once)
    (if (and *door-is-open* *rest-elevator*)
      (break))
    )

  (send *ri* :speak-jp "到着しました")


  (move-arm-press)

  (ros::ros-info "Finished.")
  )

(main)
