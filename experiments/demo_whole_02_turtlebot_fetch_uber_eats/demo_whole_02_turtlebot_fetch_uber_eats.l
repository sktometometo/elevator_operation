#!/usr/bin/env roseus

(ros::roseus-add-msgs "elevator_operation")
(ros::roseus-add-msgs "std_msgs")

(load "package://dxl_armed_turtlebot/euslisp/dxl-armed-turtlebot-interface.l")
(load "package://elevator_operation/euslisp/utils.l")


(setq *button-pressed* nil)
(defun callback-button-pressed (msg)
  (setq *button-pressed* (send msg :data))
  )

(defun wait-until-button-pressed ()
  (ros::rate 2)
  (while (ros::ok)
    (ros::sleep)
    (ros::spin-once)
    (if (*button-pressed*)
      (return))
    )
  t)


(defun main ()
  (dxl-armed-turtlebot-init)
  (ros::subscribe "/pressed" std_msgs::Bool #'callback-button-pressed)

  (setq *floor-initial* (ros::get-param "~floor_initial"))
  (setq *pose-initial* (ros::get-param "~pose_initial"))
  (setq *floor-pickup* (ros::get-param "~floor_pickup"))
  (setq *pose-pickup* (ros::get-param "~pose_pickup"))

  (send-move-base-goal
    (elt *pose-initial* 0)
    (elt *pose-initial* 1)
    (elt *pose-initial* 2)
    (elt *pose-initial* 3)
    (elt *pose-initial* 4)
    (elt *pose-initial* 5)
    (elt *pose-initial* 6)
    (elt *pose-initial* 7)
    :wait t
    )

  (wait-until-button-pressed)
  (send *ri* :speak-jp "食べ物を取りに行ってきます")

  (move-elevator *floor-pickup*)
  (send-move-base-goal
    (elt *pose-pickup* 0)
    (elt *pose-pickup* 1)
    (elt *pose-pickup* 2)
    (elt *pose-pickup* 3)
    (elt *pose-pickup* 4)
    (elt *pose-pickup* 5)
    (elt *pose-pickup* 6)
    (elt *pose-pickup* 7)
    :wait t
    )

  (send *ri* :speak-jp "食べ物をかごにいれてボタンを押して下さい")
  (wait-until-button-pressed)

  (move-elevator *floor-initial*)
  (send-move-base-goal
    (elt *pose-initial* 0)
    (elt *pose-initial* 1)
    (elt *pose-initial* 2)
    (elt *pose-initial* 3)
    (elt *pose-initial* 4)
    (elt *pose-initial* 5)
    (elt *pose-initial* 6)
    (elt *pose-initial* 7)
    :wait t
    )
  )

(main)
