#!/usr/bin/env roseus

(load "package://fetcheus/fetch-interface.l")
(load "package://jsk_fetch_startup/euslisp/navigation-utils.l")
(ros::roseus-add-srvs "elevator_operation")

(ros::roseus "look_at_elevator_server")
(fetch-init)

(setq *tfl* (instance ros::transform-listener :init))

(defun look-at-target (target-frame-id)
  (let (coords-base-to-target)
    (send *fetch* :angle-vector (send *ri* :state :potentio-vector))
    (setq coords-base-to-target (send *tfl* :lookup-transform "base_link" target-frame-id (ros::time)))
    (ros::ros-info "coords-base-to-target: ~A" coords-base-to-target)
    (if (not coords-base-to-target)
      (return-from look-at-target nil)
      )
    (if (> (elt (send coords-base-to-target :pos) 0) 0)
          (progn
            (send *fetch* :look-at-target coords-base-to-target)
            )
        (let* ((target-x (elt (send coords-base-to-target :pos) 0))
               (target-y (elt (send coords-base-to-target :pos) 1)))
          (send *ri* :go-pos-unsafe 0 0 (rad2deg (atan target-y target-x)))
          (send *fetch* :angle-vector (send *ri* :state :potentio-vector))
          (setq coords-base-to-target (send *tfl* :lookup-transform "base_link" target-frame-id (ros::time)))
          (if (not coords-base-to-target)
            (return-from look-at-target nil)
            )
          (send *fetch* :look-at-target coords-base-to-target)
          )
        )
    (send *ri* :angle-vector-raw (send *fetch* :angle-vector) 500)
    t
    )
  )

(defun handler-start-look-at-target (req)
  (let (success
        (res (instance elevator_operation::LookAtTargetResponse :init)))
    (setq success (look-at-target (send req :frame_id)))
    (ros::ros-warn "makabe")
    (send res :success success)
    res))

(ros::advertise-service "~look_at_target" elevator_operation::LookAtTarget #'handler-start-look-at-target)
(ros::spin)
