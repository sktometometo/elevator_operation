#!/usr/bin/env roseus

(load "package://fetcheus/fetch-interface.l")
(load "package://jsk_fetch_startup/euslisp/navigation-utils.l")
(ros::roseus-add-srvs "std_srvs")

(ros::roseus "look_at_elevator_server")
(fetch-init)

(setq *tfl* (instance ros::transform-listener :init))
(setq *target-frame-id* (ros::get-param "~target_frame_id" "hoge"))

(defun look-at-target ()
  (let (coords-base-to-target)
    (send *fetch* :angle-vector (send *ri* :state :potentio-vector))
    (setq coords-base-to-target (send *tfl* :lookup-transform "base_link" *target-frame-id* (ros::time)))
    (setq ret (send *fetch* :look-at-target coords-base-to-target))
    (if (not ret)
        (let* ((target-x (elt (send coords-base-to-target :pos) 0))
               (target-y (elt (send coords-base-to-target :pos) 1)))
          (send *ri* :go-pos-unsafe 0 0 (rad2deg (atan target-y target-x)))
          (send *fetch* :angle-vector (send *ri* :state :potentio-vector))
          (setq ret (send *fetch* :look-at-target coords-base-to-target))
          )
        nil
        )
    (send *ri* :angle-vector-raw (send *fetch* :angle-vector) 500)
    t
    )
  )

(defun handler-start-look-at-target (req)
  (let (success
        (res (instance std_srvs::Trigger :init)))
    (setq success (look-at-target))
    (send res :success success)
    res
    )
  )

(ros::advertise-service "~look_at_target" std_srvs::Trigger #'handler-start-look-at-target)
(ros::spin)
