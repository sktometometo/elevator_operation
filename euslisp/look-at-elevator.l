#!/usr/bin/env roseus

(load "package://fetcheus/fetch-interface.l")
(load "package://jsk_fetch_startup/euslisp/navigation-utils.l")

(fetch-init)

(setq *tfl* (instance ros::transform-listener :init))
(setq *target-frame-id* (ros::get-param "~target_frame_id" "hoge"))

(while (ros::ok)
  (let (coords-base-to-target)
  (send *fetch* :angle-vector (send *ri* :state :potentio-vector))
  (setq coords-base-to-target (send *tfl* :lookup-transform "base_link" "elevator_frame" (ros::time)))
  (send *fetch* :look-at-target coords-base-to-target)
  (send *ri* :angle-vector-raw (send *fetch* :angle-vector) 500)
  )
