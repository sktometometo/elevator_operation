#!/usr/bin/env roseus

(ros::roseus-add-srvs "elevator_operation")

(ros::roseus "look_at_elevator_server")
(setq *robot-type* (ros::get-param "~robot_type"))

(cond
  ((eq *robot-type* "fetch")
    (load "package://fetcheus/fetch-interface.l")
    (load "package://jsk_fetch_startup/euslisp/navigation-utils.l")
    (fetch-init)
    (setq *robot* *fetch*))
  ((eq *robot-type* "magni")
    (setq *move-base-action-client* (instance ros::simple-action-client :init "/move_base" move_base_msgs::MoveBaseAction))
    (send *move-base-action-client* :wait-for-server)
    )
  (t
    (ros::ros-error "Unsupported robot type: ~A" *robot-type*))
  )
(setq *tfl* (instance ros::transform-listener :init))

(defun update-model ()
  (if (eq *robot-type "fetch")
    (send *robot* :angle-vector (send *ri* :state :potentio-vector)))
  )

(defun look-at-target-with-head (coords-base-to-target)
  (if (eq *robot-type "fetch")
    (progn
      (send *robot* :look-at-target coords-base-to-target)
      (send *ri* :angle-vector-raw (send *robot* :angle-vector) 500)))
  )

(defun rotate-base (theta)
  (cond
    ((eq *robot-type* "fetch")
      (send *ri* :go-pos-unsafe-wait 0 0 (rad2deg theta)))
    ((eq *robot-type* "magni")
      (let ((goal (instance move_base_msgs::MoveBaseGoal :init)))
        (send goal :target_pose :header :frame_id "base_link")
        (send goal :target_pose :pose :orientation :z (sin (/ theta 2)))
        (send goal :target_pose :pose :orientation :w (cos (/ theta 2)))
        (send *move-base-action-client* :send-goal goal)
        (send *move-base-action-client* :wait-for-result)
        ))
    (t
      (ros::ros-error "Unsupported robot type: ~A" *robot-type*))
    ))

(defun look-at-target-frame (target-frame-id)
  (let (coords-base-to-target)
    (update-model)
    (setq coords-base-to-target (send *tfl* :lookup-transform "base_link" target-frame-id (ros::time)))
    (ros::ros-info "coords-base-to-target: ~A" coords-base-to-target)
    (if (not coords-base-to-target)
      (return-from look-at-target nil)
      )
    (let* ((target-x (elt (send coords-base-to-target :pos) 0))
           (target-y (elt (send coords-base-to-target :pos) 1)))
      (rotate-base (rad2deg (atan target-y target-x)))
      (update-model)
      (setq coords-base-to-target (send *tfl* :lookup-transform "base_link" target-frame-id (ros::time)))
      (if (not coords-base-to-target)
        (return-from look-at-target nil)
        )
      (look-at-target-frame coords-base-to-target)
      )
    t))

(defun handler-start-look-at-target (req)
  (let (success
        (res (instance elevator_operation::LookAtTargetResponse :init)))
    (setq success (look-at-target-frame (send req :frame_id)))
    (ros::ros-warn "makabe")
    (send res :success success)
    res))

(ros::advertise-service "~look_at_target" elevator_operation::LookAtTarget #'handler-start-look-at-target)
(ros::spin)
