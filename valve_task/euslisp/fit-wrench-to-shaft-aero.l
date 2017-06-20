(load "package://valve_task/models/panel.l")
(load "package://valve_task/models/wrench.l")
(load "package://aero_utils/euslisp/aero-arm-single-wrist-utils.l")
(load "package://aero_dgripper/euslisp/aero-dgripper-controller.l")
(load "package://valve_task/models/task2arena-scene.l")
(load "package://valve_task/euslisp/valve-task-params.l")
(load "package://valve_task/euslisp/aero-arm-turn-wrench-torso.l")

;;;;;;;;;; state flow ;;;;;;;;;;
;;   turn-shaft-full-turn
;;             |
;;    fit-wrench-to-shaft
;;             |
;; detect-shaft-center-state
;;             |
;;;;;      slide-to-fit-state
;;;;;             |
;;             |
;;     turn-shaft-state

(setq arm :rarm)

(setq turn-radius wrenchLength)

(setq debug t)

(setq wrenchFittingAlignAngle 0)

(defun prompt (msg)
    (if debug
            (progn
                (print msg)
                (read-char)))
    )

(defun turn-shaft-full-turn ()

    (format t "enter turn-shaft-full-turn~%")
    (setq turn-shaft-start-arm-pose (send *robot* arm :end-coords :copy-worldcoords))
    (setq start-pose-delta 0)
    (setq turn_count 1)
    (setq start-pose-delta (fit-wrench-to-shaft start-pose-delta))
    (while (< turn_count 600)
        (unless start-pose-delta
            (return-from turn-shaft-full-turn nil))

        (if start-pose-delta ;; if successfully turned without error
                (progn
                    (setq wrench-normal-vector ;; in the direction towards the shaft, in lifter frame
                          (float-vector
                           (sin (deg2rad (get-gripper-pos)))
                           0
                           (- (cos (deg2rad (get-gripper-pos)))))
                          )
                    (format t "detaching wrench")
                    (setq detach-wrench-move-waist-distance 40)
                    (movewaist (scale detach-wrench-move-waist-distance (v- wrench-normal-vector)))
                    (gripper-servo-enable)
                    (gripper-servo-pos 0)
                    (format t "move arm back to turn-shaft-start-arm-pose")
                    (format t "turn-shaft-start-arm-pose ~a~%" turn-shaft-start-arm-pose)
                    (send *robot* arm :inverse-kinematics turn-shaft-start-arm-pose)
                    (moverobot 4000)
                    (format t "move lifter back to start position")
                    (movewaist #f(150 0 -50) :world 4000) 
                    (unix:sleep 4)
                    (setq start-pose-delta (fit-wrench-to-shaft start-pose-delta))
                    (setq turn_count (+ turn_count 1))
                    )
            (progn ;; if failed to turn
                (print "need to implement recovery method")
                (return-from turn-shaft-full-turn nil)
                )
            )
        )
    (format t "detaching wrench")
    (setq detach-wrench-move-waist-distance 40)
    (movewaist (scale detach-wrench-move-waist-distance (v- wrench-normal-vector)))
    )

(defun turn-shaft-disengage-move-arm ()

    (format t "enter turn-shaft-full-turn~%")
    (matchsimwithrobot)
    (setq turn-shaft-start-arm-pose (send *robot* arm :end-coords :copy-worldcoords))
    (format t "1 turn-shaft-start-arm-pose ~a~%" turn-shaft-start-arm-pose)
    (setq start-pose-delta 0)

    (turnWrenchTorso3)


    (setq wrench-normal-vector ;; in the direction towards the shaft, in lifter frame
          (float-vector
           (sin (deg2rad (get-gripper-pos)))
           0
           (- (cos (deg2rad (get-gripper-pos)))))
          )
    (format t "detaching wrench")
    (setq detach-wrench-move-waist-distance 40)
    (movewaist (scale detach-wrench-move-waist-distance (v- wrench-normal-vector)))
    (gripper-servo-enable)
    (gripper-servo-pos 0)
    (format t "move arm back to slightly above turn-shaft-start-arm-pose")
    (format t "2 turn-shaft-start-arm-pose ~a~%" turn-shaft-start-arm-pose)
    (send *robot* arm :inverse-kinematics (send turn-shaft-start-arm-pose :translate #f(0 0 50)))
    (moverobot 4000)
    (format t "move lifter back to start position")
    (movewaist #f(150 0 -50) :world 4000) 
    (unix:sleep 4)
    )


;; Entry point to the entire state machine!
(defun fit-wrench-to-shaft (&optional prev-arm-pose-delta)
    (format t "Entered fit-wrench-to-shaft.~%")

    (gripper-servo-enable)
    (gripper-servo-pos wrenchFittingAlignAngle 1)

    (setq approach-vector #f(0 0 -1)) ;; local frame wrt the hand
    (setq tangent-vector #f(0 1 0)) ;; local frame wrt the hand

    ;;;;;;;;;;;;; here's the state machine! ;;;;;;;;;;;;;;
    ;;;;;;; state : scan-for-contact-state ;;;;;;;;;;;
    (unless (scan-for-contact-state 0)
        (return-from fit-wrench-to-shaft nil))
    
    ;; remember state at initial contact
    (setq arm-end-coords-at-initial-contact (send *robot* arm :end-coords :copy-worldcoords))
    (setq tangent-force (v. (get-offset-force-vector) tangent-vector))
    (setq approach-force (v. (get-offset-force-vector) approach-vector))

    ;; (twist-to-fit-state)

    ;;;;;;; state : detect-shaft-center-state ;;;;;;;;;;;;;
    (setq detect-shaft-center-result (detect-shaft-center-state))
    (unless  detect-shaft-center-result
        (return-from fit-wrench-to-shaft nil))
    ;; (setq detect-shaft-center-result 0)


    (format t "detect-shaft-center-result ~a~%" detect-shaft-center-result)

    (if (boundp `prev-arm-pose-delta)
            (setq start-arm-pose-delta prev-arm-pose-delta)
        (setq start-arm-pose-delta 0)
        )
    (while (not (eq detect-shaft-center-result 0))
        (unless detect-shaft-center-result
          (progn 
            (format t "detect-shaft-center-result returned nil. Wrench head is not touching shaft at all!")
            (return-from fit-wrench-to-shaft nil)
            )
          )
        (format t "detect-shaft-center-result did not return 0~%")
        (format t "scan-for-contact-state-start-pose ~a~%" scan-for-contact-state-start-pose) 
        (send *robot* arm :inverse-kinematics scan-for-contact-state-start-pose)
        (if (boundp `*irtviewer*)
                (send *irtviewer* :draw-objects))
        (moverobot 1000)
        (send *ri* :wait-interpolation)
        (unix:sleep 2) ;; wait for force sensor values to settle
        (if (< detect-shaft-center-result 0) ;; negative means detected force while moving left 
                (setq current-delta (* 3 (exp (exp (- (abs detect-shaft-center-result))))))
            (setq current-delta (- (* 3 (exp (exp (- (abs detect-shaft-center-result)))))))
            )
        (setq start-arm-pose-delta (+ start-arm-pose-delta current-delta))
        (format t "detect-shaft-center-result ~a. start-arm-pose-delta now ~a~%" detect-shaft-center-result start-arm-pose-delta)

        ;; here's the state!
        (setq scan-for-contact-state-result (scan-for-contact-state start-arm-pose-delta))
        (if (not scan-for-contact-state-result)
                (return-from fit-wrench-to-shaft nil))
        (setq detect-shaft-center-result (detect-shaft-center-state))
        (unless detect-shaft-center-result
            (return-from fit-wrench-to-shaft nil))
        (format t "detect-shaft-center-result ~a~%" detect-shaft-center-result)
        
        )
    (print "entering turn-shaft-state from fit-wrench-to-shaft-state")

    ;;;;;;;; state : turn-shaft-state ;;;;;;;;;
    (turn-shaft-state)

    (print "exited from turn-shaft-state back to fit-wrench-to-shaft-state")
    start-arm-pose-delta ;; return the arm-pose delta of where the shaft was detected to be used for next call to fit-wrench-to-shaft
    )

;; STATE: scan for contact
;; move in approach-vector direction until touched something 
(defun scan-for-contact-state (start-pos-delta)
       (format t "Entered scan-for-contact-state with start-pos-delta ~a~%" start-pos-delta)
       (unix:sleep 2)
       (get-force-offset)
       (setq touch-lower-thres 1.0)
       (setq touch-upper-thres 3.5)
       
       (setq target-pose (send (send *robot* arm :end-coords :copy-worldcoords) :translate (float-vector 0 start-pos-delta 0)))
       (setq scan-for-contact-state-start-pose (send target-pose :copy-worldcoords))

       (setq approach-scan-distance 150)
       (setq scan-for-contact-state-tangent-scan-distance 10)
       (setq approach-inc-dist 1.0)
       (setq scan-for-contact-state-tangent-inc-dist 3.0)       
       
       (setq tangent-index 0) ;; index for scanning in the tangent direction
       (setq target-pose (send *robot* arm :end-coords :copy-worldcoords))
       (while (< tangent-index (* 2 (/ scan-for-contact-state-tangent-scan-distance scan-for-contact-state-tangent-inc-dist)))
           (format t "tangent-index ~a~%" tangent-index)
           (setq target-pose (send scan-for-contact-state-start-pose :copy-coords))
           (send target-pose :translate (float-vector 0 start-pos-delta 0))
           (if (equal 0 (mod tangent-index 2)) ;; do a widening zigzag scan in the tangent direction 
                   (send target-pose :translate (scale (* (/ tangent-index 2) scan-for-contact-state-tangent-inc-dist) tangent-vector))
               (send target-pose :translate (scale (- (* (/ tangent-index 2) scan-for-contact-state-tangent-inc-dist)) tangent-vector)))
           (send *robot* arm :inverse-kinematics target-pose)
           (moverobot 300)
           (send *ri* :wait-interpolation)
           (setq approach-index 0) ;; index for scanning in the approach direction 
           (while (< approach-index (/ approach-scan-distance approach-inc-dist))
                   ;; (prompt "scan-for-contact-state: hit enter to step through")
               (send target-pose :translate (scale approach-inc-dist approach-vector))
               (send *robot* arm :inverse-kinematics target-pose)
               (moverobot 500)
               ;; (send *ri* :wait-interpolation)
                   (if (boundp '*irtviewer*)
                           (send *irtviewer* :draw-objects))
                   (unix:usleep 250000)
                   (format t "offset force ~a~%" (get-offset-force-vector))
                   (if (> (abs (v. (v- (get-force-vector) force-offset) approach-vector)) touch-lower-thres) ;; if touch something 
                       (progn
                         (unix:usleep 1000000) ;; wait for two seconds to account for possible slip of wrench on shaft
                         (if (> (abs (v. (v- (get-force-vector) force-offset) approach-vector)) touch-lower-thres) ;; check again to see if really touching something
                             (progn
                               (print "touch-lower-thres reached.")
                               (print "now moving back in opposite direction to achieve minimum touch force.")
                               ;; move back in the opposite direction to maintain minimum force 
                               ;; this is to avoid pressing too hard on the shaft so the wrench head can't slide on it
                               (unix::usleep 300000) ;; wait for force reading to settle
                               (setq approach-force (v. (get-offset-force-vector) approach-vector))
                               (while (> (abs approach-force) touch-lower-thres)
                                 (send target-pose :translate (scale (- approach-inc-dist) approach-vector))
                                 (send *robot* arm :inverse-kinematics target-pose)
                                 (moverobot 500)
                                 (unix::usleep 300000)
                                 (setq approach-force (v. (get-offset-force-vector) approach-vector))
                                 (format t "approach-force ~a~%" approach-force)

                                 )
                               (print "Minimum touch force reached. Exit scan-for-contact-state")

                               ;; (prompt "hit enter to continue...")
                               (return-from scan-for-contact-state t))
                           )
                         )
                     )
                   (inc approach-index)

                   ;; check absolute height of hand
                   (setq shaftHeight 980)
                   (setq fittingPoseLifterHeight 1110)
                   (setq armEndHeight (elt (send *robot* :rarm :end-coords :worldpos) 2))
                   
                   (setq wrenchHeadHeight
                         (- (+ fittingPoseLifterHeight armEndHeight)
                            wrenchLength
                            20 ;; wrenchLength is measured from ring end center to open end center, so add approximately 2 cm to get full length
                            )
                         )
                   (if (< wrenchHeadHeight
                          (- shaftHeight
                             60) ;; error tollerance in calculating wrenchheadheight
                          )
                           (progn
                               (format t "----- wrench head height too low. Must have missed shaft completely -----~%")
                               (format t "~%")
                               (format t "~%")
                               (return-from scan-for-contact-state nil)
                               )
                       )
                   
                   
                   )
           (inc tangent-index)
           )
       nil
       ) ;; end STATE: scan for contact

(defun detect-shaft-center-state ()
    (format t "Entered detect-shaft-center-state.~%")
    (format t "offset force: ~a~%" (get-offset-force-vector))
    (setq force-vector (get-offset-force-vector))
    (setq tangent-force (v. force-vector tangent-vector)) 
    (slide-to-fit-state)
    )


;; STATE: slide to fit
;; Once the wrench has detected contact with force in the approach direciton, but no force in the tangent direction,
;; slide hand left and right while maintaining a downward force to try to ensure/get a better fitting of the wrench to the shaft
(defun slide-to-fit-state ()
    (get-force-offset)
    (setq safety-force-limit 70)
    (setq shaft-width 19)

    (format t "Entered slide-to-fit-state~%")
    ;; (prompt "hit enter to continue")
    (setq start-pose (send *robot* arm :end-coords :copy-worldcoords))    
    (setq target-pose (send *robot* arm :end-coords :copy-worldcoords))
    
    (setq force-sensor-wait-time 1000)

    (setq tangent-scan-distance 40.0)
    (setq tangent-inc-dist 2.0)       
    (setq approach-inc-dist 2.0)
    (setq tangent-force-thres 1.0)

    
    (Setq Detected-Force-Left Nil)
    (Setq Detected-Force-Right Nil)
    (setq tangent-index 0) ;; Index For Scanning In The Tangent Direction
    (Setq Loop-Start-Pose (Send Start-Pose :Copy-coords))
    (print "loop-start-pose")
    (print loop-start-pose)
    (print "")
    (print "")
    (setq last-loop-state nil)
    (while (<= tangent-index (* 2 (/ tangent-scan-distance tangent-inc-dist)))

        (setq target-pose (send loop-start-pose :copy-coords))

        (setq force-vector (get-offset-force-vector))
        (setq tangent-force (v. force-vector tangent-vector)) 
        (setq approach-force (v. force-vector approach-vector)) 

        (setq had-to-servo-force nil)
        (format t "******** while     loop      begin *********~%")
        (format t "scan tangent index ~d / ~d~%" tangent-index (* 2 (/ tangent-scan-distance tangent-inc-dist)))
        (format t "force-vector ~a~%" force-vector)
        (format t "tangent-force ~a~%" tangent-force)
        (format t "approach-force ~a~%" approach-force)
        (format t "~%")
        (format t "~%")

        ;; first mantain a contact force in the approach direction

        ;; servo force in approach direction

        (if (and (< (abs approach-force) touch-lower-thres) (not (equal last-loop-state "force-servo-dec"))) ;; force servoing in the approach direction
                (progn ;; if force below threshold
                  (while (< (abs approach-force) touch-lower-thres)
                    (format t "------ force servoing ------~%")
                    (format t "approach-force below touch-lower-thres. Adjust target-pos.~%")
                    (setq delta (scale approach-inc-dist approach-vector))
                    (send target-pose :translate (scale approach-inc-dist approach-vector))
                    (send loop-start-pose :translate (scale approach-inc-dist approach-vector))
                    (format t "target pos ~a~%" (send target-pose :pos))
                    (format t "loop start pos ~a~%" (send loop-start-pose :pos))
                    (format t "~%")
                    
                    ;; check if hand moved too far in approach direction

                    (if (> (abs (v. (v- (send start-pose :pos) (send target-pose :pos)) approach-vector)) ;; check relative to start position 
                           (* 2 shaft-width))
                        (progn
                          (format t "----- moved too far. Must have missed shaft completely -----~%")
                          (format t "~%")
                          (format t "~%")
                          (return-from slide-to-fit-state (slide-to-fit-state-return-helper))
                          )
                        
                      )
                    

                    ;; (prompt "hit enter to move arm")
                    (slide-to-fit-move-arm delta)
                    ;; (send *robot* arm :inverse-kinematics target-pose)
                    ;; (moverobot slide-to-fit-moverobot-time)
                    ;; (send *ri* :wait-interpolation)
                    (if (> (norm (get-offset-force-vector)) safety-force-limit)
                            (return-from slide-to-fit-state slide-to-fit-state-return-helper))                            

                    (unix:usleep force-sensor-wait-time)
                    (setq force-vector (get-offset-force-vector))
                    (format t "get-offset-force-vector ~a~%" (get-offset-force-vector))
                    (setq current-tangent-force (v. force-vector tangent-vector))
                    (setq approach-force (v. force-vector approach-vector))
                    (format t "~%")
                    (format t "force-vector ~a~%" force-vector)
                    (format t "approach-force ~a~%" approach-force)
                    (format t "~%")
                    (format t "~%")
                    )
                  (format t "servoed force, set had-to-servo-force to t~%")
                  (setq had-to-servo-force t)
                  (setq last-loop-state "force-servo-inc")
                  )  ;; end if force below threshold
          (progn  ;; else if force above threshold
            (if (and (> (abs approach-force) touch-upper-thres) (not (equal last-loop-state "force-servo-inc"))) ;; force servoing in the approach direction
              (progn
                  (while (> (abs approach-force) touch-upper-thres)
                    (format t "+++++++ force servoing ++++++~%")
                    (format t "approach-force above touch-upper-thres. Adjust target-pos.~%")
                    (setq delta (scale (- approach-inc-dist) approach-vector))
                    (send target-pose :translate (scale (- approach-inc-dist) approach-vector))
                    (send loop-start-pose :translate (scale (- approach-inc-dist) approach-vector))
                    (format t "target pos ~a~%" (send target-pose :pos))
                    (format t "loop start pos ~a~%" (send loop-start-pose :pos))
                    (format t "~%")

                    ;; check if hand moved too far in approach direction
                    (if (> (abs (v. (v- (send start-pose :pos) (send target-pose :pos)) approach-vector))
                           (* 2 shaft-width))
                        (progn
                          (format t "----- moved too far. Must have missed shaft completely -----~%")
                          (format t "~%")
                          (format t "~%")
                          (return-from slide-to-fit-state (slide-to-fit-state-return-helper))
                          )
                      )
                    ;; check absolute height of hand
                    (setq shaftHeight 980)
                    (setq fittingPoseLifterHeight 1110)
                    (setq armEndHeight (elt (send *robot* :rarm :end-coords :worldpos) 2))

                    (setq wrenchHeadHeight
                          (- (+ fittingPoseLifterHeight armEndHeight)
                             wrenchLength
                             20 ;; wrenchLength is measured from ring end center to open end center, so add approximately 2 cm to get full length
                             )
                          )
                    (if (< wrenchHeadHeight
                           (- shaftHeight
                              60) ;; error tollerance in calculating wrenchheadheight
                           )
                        (progn
                          (format t "----- wrench head height too low. Must have missed shaft completely -----~%")
                          (format t "~%")
                          (format t "~%")
                          (return-from slide-to-fit-state (slide-to-fit-state-return-helper))
                          )
                        )
                    
                    ;; (prompt "hit enter to mov arm")
                    (slide-to-fit-move-arm delta)
                    ;; (send *robot* arm :inverse-kinematics target-pose)
                    ;; (moverobot slide-to-fit-moverobot-time)
                    ;; (send *ri* :wait-interpolation)
                    (if (> (norm (get-offset-force-vector)) safety-force-limit)
                            (return-from slide-to-fit-state (slide-to-fit-state-return-helper)))

                    (unix:usleep force-sensor-wait-time)
                    (setq force-vector (get-offset-force-vector))
                    (setq current-tangent-force (v. force-vector tangent-vector))
                    (setq approach-force (v. force-vector approach-vector))
                    (format t "force-vector ~a~%" force-vector)
                    (format t "approach-force ~a~%" approach-force)
                    (format t "~%")
                    (format t "~%")
                    )
                  (format t "servoed force, set had-to-servo-force to t~%")
                  (setq had-to-servo-force t)
                  (setq last-loop-state "force-servo-dec")
                  )
              )
            ) ;; end else if force above threshold
          ) ;; end if force below threshold

        ;; after force has been served to the right amount
        ;; check if detected shaft left or right
        (unless had-to-servo-force
          (if (< tangent-force (- tangent-force-thres)) ;; if detected force when moving left (force negative value)
              (progn
                (format t "detected force when moving left~%")
                (format t "~%")
                (if detected-force-right ;; if has already detected force when moving right (force positive value)
                    (progn
                      (format t "----- moving back right to achieve near zero tangent force ------~%")
                      (setq current-tangent-force tangent-force)
                      (format t "current-tangent-force ~a~%" current-tangent-force)
                      (while (< current-tangent-force (- tangent-force-thres)) ;; move back right to achieve near zero tangent force
                        (format t "offset force: ~a~%" (get-offset-force-vector))
                        ;; (prompt "moving back to shaft center....")
                        (setq delta (scale (- tangent-inc-dist) tangent-vector)) ;; move hand right a bit
                        (send target-pose :translate (scale (- tangent-inc-dist) tangent-vector)) ;; move hand right a bit


                        (if (< (abs approach-force) touch-lower-thres) ;; force servoing in the approach direction
                            (progn
                              (format t "approach-force below touch-lower-thres. Adjust target-pos.~%")
                              (setq delta (scale approach-inc-dist approach-vector))
                              (send target-pose :translate (scale approach-inc-dist approach-vector))
                              (send loop-start-pose :translate (scale approach-inc-dist approach-vector))
                              (format t "target pos ~a~%" (send target-pose :pos))
                              )
                          )
                        ;; check if hand moved too far
                        (if (> (v. (v- (send start-pose :pos) (send target-pose :pos)) approach-vector)
                               (* 2 shaft-width))
                            (progn
                              (format t "----- moved too far while centering. Must have missed shaft completely -----~%")
                              (format t "~%")
                              (format t "~%")
                              (return-from slide-to-fit-state (slide-to-fit-state-return-helper))
                              )
                          )

                        ;; (prompt "hit enter to move arm")
                        (slide-to-fit-move-arm delta)
                        ;; (send *robot* arm :inverse-kinematics target-pose)
                        ;; (moverobot slide-to-fit-moverobot-time)
                        ;; (send *ri* :wait-interpolation)
                        (if (> (norm (get-offset-force-vector)) safety-force-limit)
                                (progn
                                    (format t "force limit over!!! stopping")
                                    (return-from slide-to-fit-state nil))
                            )

                        (unix:usleep force-sensor-wait-time)
                        (setq current-tangent-force (v. (get-offset-force-vector) tangent-vector))
                        (format t "current-tangent-force ~a~%" current-tangent-force)
                        (setq apporach-force (v. (get-offset-force-vector) approach-vector))
                        )
                      (return-from slide-to-fit-state (confirm-shaft-center-found))
                      )
                  (setq detected-force-left t)) ;; else if has not yet detected force when moving right
                )
            )
          (if (> tangent-force tangent-force-thres) ;; if detected force when moving right 
              (progn
                (format t "detected force when moving right~%") ;; force positive value
                (format t "~%")
                (if detected-force-left ;; if has already detected force when moving left (negative value)
                    (progn
                      (format t "------ moving back left to achieve near zero tangent force ------~%")
                      (setq current-tangent-force tangent-force)
                      (format t "current-tangent-force ~a~%" current-tangent-force)
                      (while (> current-tangent-force tangent-force-thres) ;; move back left to achieve near zero tangent force
                        (format t "offset force: ~a~%" (get-offset-force-vector))
                        ;; (prompt "moving back to shaft center....")
                        (setq delta (scale tangent-inc-dist tangent-vector)) ;; move hand left a bit        
                        (send target-pose :translate (scale tangent-inc-dist tangent-vector)) ;; move hand left a bit
                        
                        (if (< (abs approach-force) touch-lower-thres) ;; force servoing in the approach direction
                            (progn
                              (format t "approach-force below touch-lower-thres. Adjust target-pos.~%")
                              (setq delta (scale approach-inc-dist approach-vector))
                              (send target-pose :translate (scale approach-inc-dist approach-vector))
                              (send loop-start-pose :translate (scale approach-inc-dist approach-vector))
                              (format t "target pos ~a~%" (send target-pose :pos))
                              )
                          )

                        ;; check if hand moved too far
                        (if (> (v. (v- (send start-pose :pos) (send target-pose :pos)) approach-vector)
                               (* 2 shaft-width))
                            (progn
                              (return-from slide-to-fit-state (slide-to-fit-state-return-helper))
                              )
                          )
                        
                        ;; (prompt "hit enter to move arm")
                        (slide-to-fit-move-arm delta)
                        ;; (send *robot* arm :inverse-kinematics target-pose)
                        ;; (moverobot slide-to-fit-moverobot-time)
                        ;; (send *ri* :wait-interpolation)
                        (if (> (norm (get-offset-force-vector)) safety-force-limit)
                                (progn
                                    (format t "Force limit over!!! Stopping")
                                    (return-from slide-to-fit-state nil))
                            )

                        (unix:usleep force-sensor-wait-time)
                        (setq current-tangent-force (v. (get-offset-force-vector) tangent-vector))
                        (format t "current-tangent-force ~a~%" current-tangent-force)
                        (setq apporach-force (v. (get-offset-force-vector) approach-vector))
                        )
                      (return-from slide-to-fit-state (confirm-shaft-center-found))
                      )
                  (setq detected-force-right t)) ;; else if has not yet detected force when moving right
                )
            )
          
          ;; check if hand moved too far in approach direction
          (if (> (v. (v- (send start-pose :pos) (send target-pose :pos)) approach-vector)
                 (* 2 shaft-width))
              (progn
                (format t "----- moved too far. Must have missed shaft completely -----~%")
                (format t "~%")
                (format t "~%")
                (return-from slide-to-fit-state (slide-to-fit-state-return-helper))
                )
            )
          
          (format t "------ zig zaging ------~%")
          (if (equal 0 (mod tangent-index 2)) ;; do a widening zigzag scan in the tangent direction 
              (progn
                (format t "move left~%")
                (setq delta (scale (* (/ tangent-index 2) tangent-inc-dist) tangent-vector))
                (send target-pose :translate (scale (* (/ tangent-index 2) tangent-inc-dist) tangent-vector))
                (setq last-loop-state "zig-zag-left")
                )
            (progn
              (format t "move right~%")
              
              (setq delta (scale (- (* (/ tangent-index 2) tangent-inc-dist)) tangent-vector))
              (send target-pose :translate (scale (- (* (/ tangent-index 2) tangent-inc-dist)) tangent-vector))
              (setq last-loop-state "zig-zag-right")
              )
            )
          
          (format t "target pos ~a~%" (send target-pose :pos))
          (format t "loop start pos ~a~%" (send loop-start-pose :pos))
          (format t "~%")
          (format t "~%")

          ;; (prompt "hit enter to move arm")
          (slide-to-fit-move-arm delta)
          ;; (send *robot* arm :inverse-kinematics target-pose)
          ;; (moverobot slide-to-fit-moverobot-time)
          ;; (send *ri* :wait-interpolation)
          (if (> (norm (get-offset-force-vector)) safety-force-limit)
                  (progn
                      (format t "Force limit over!!! Stopping")
                      (return-from slide-to-fit-state nil))
              )

          (unix:usleep force-sensor-wait-time)
          (setq current-tangent-force (v. (get-offset-force-vector) tangent-vector))
          (setq apporach-force (v. (get-offset-force-vector) approach-vector))
          
          
          (inc tangent-index) 
          )
        
        ) ;; end doing zig zag scan 

    ;; done zig zag scan, but still can't find shaft center, so return to start pose
    (move-arm #f(0 0 30))
    (send *robot* arm :inverse-kinematics start-pose)
    (moverobot 2000)
    (send *ri* :wait-interpolation)
    (unix:sleep 2)
    
    (slide-to-fit-state-return-helper)

    ) ;; end STATE: slide to fit

(defun slide-to-fit-state-return-helper ()

;;    (return-from slide-to-fit-state-return-helper (confirm-shaft-center-found))
    (format t "In slide-to-fit-state-return-helper~%")
    (format t "detected-force-right ~a~%" detected-force-right)
    (format t "detected-force-left ~a~%" detected-force-left)
    (if (and (not detected-force-right) (not detected-force-left))
            (return-from slide-to-fit-state-return-helper nil))
    (cond
     (detected-force-right 1)
     (detected-force-left -1)
     (t nil)
     )
    )


;; STATE: twist to fit 
;; Twist the wrench left and right while pushing down to get it.
(defun twist-to-fit-state ()
    
    (return-from twist-to-fit-state)
    
    (setq consecutive-peak-increase 3.0) ;; how much the peak increase each time
    (setq next-peak-amplitude consecutive-peak-increase)
    (setq max-peak-amplitude 30.0) ;; maximum amount of yaw to twist
    (setq yaw-angle-inc 3.0) ;; amount of yaw to twist in each step of the loop
    (setq twist-axis #f(0 0 1)) ;; local frame wrt the hand
    (setq current-twist-angle 0) ;; just for keeping track of the current twist angle
    (setq target-pose (send *robot* arm :end-coords :copy-worldcoords))
    (while (< current-twist-angle max-peak-amplitude)
        (prompt "twist-to-fit-state: hit enter to step through")
        (if (> next-peak-amplitude 0) ;; yaw angle increasing 
                (progn
                    (print "(> next-peak-amplitude 0)")
                    (if (< current-twist-angle next-peak-amplitude)
                            (progn ;; current angle hasn't reached next peak yet
                                (send target-pose :rotate (deg2rad yaw-angle-inc) twist-axis) ;; continue increasing
                                (setq current-twist-angle (+ current-twist-angle yaw-angle-inc))
                                )
                        (progn ;; current angle has reached next peak                                   
                            (setq next-peak-amplitude (- next-peak-amplitude))
                            )
                        )
                    )
            (progn ;; yaw angle is decreasing
                (if (> current-twist-angle next-peak-amplitude) 
                        (progn ;; current angle hasn't reached next peak yet
                            (send target-pose :rotate (- (deg2rad yaw-angle-inc)) twist-axis) ;; continue decreasing
                            (setq current-twist-angle (- current-twist-angle yaw-angle-inc))
                            )
                    (progn ;; current angle has reached next peak                                   
                        (setq next-peak-amplitude (+ (- next-peak-amplitude) consecutive-peak-increase))
                        )
                    )
                )
            )
        (if (< (abs (v. (get-offset-force-vector) approach-vector)) touch-lower-thres) ;; try to maintain a downwards force
                (send target-pose :translate (scale approach-inc-dist approach-vector))
            )
        
        (send *robot* arm :inverse-kinematics target-pose)
        (moverobot 300)
        (send *ri* :wait-interpolation)
        (if (boundp '*irtviewer*)
                (send *irtviewer* :draw-objects))
        (unix:usleep 5000)
        
        (setq offset-force-vector (get-offset-force-vector))
               (setq approach-force-vector (scale (v. (v- (get-force-vector) force-offset) approach-vector) approach-vector))
               (setq tangent-force (v- offset-force-vector approach-force-vector))
               (format t "offset force ~a~%" offset-force-vector)
               
               (if (> (norm tangent-force) touch-lower-thres) ;; if touch something 
                       (progn
                           (print "tangent force exceeded threshold. Exit twist-to-fit-state")
                           (return-from twist-to-fit-state)
                           )
                   )
               )
    )  ;; end STATE: twist to fit 

;; STATE: turn shaft state
;; Turn the wrech in a clockwise trajectory after the wrench has fitted to the shaft
(defun turn-shaft-state ()
    (format t "Entered turn-shaft-state.~%")
    (gripper-servo-disable)
    ;; (unless (turn-wrench-torso2 (send (send *robot* :rarm :end-coords :copy-worldcoords) :translate (float-vector 0 0 (- turn-radius)) :world) (- turn-radius 0))
    (unless (turnWrenchTorso3)
      (progn
        (format t "Turn shaft failed.~%")
        (return-from turn-shaft-state)
        )
      )

    (format t "Turn shaft done.~%")
    ;; (prompt "Hit enter to move hand right...")
;    (send *robot* arm :inverse-kinematics (send (send *robot* arm :end-coords :copy-worldcoords) :translate #f(0 -50 0) :world))
;    (moverobot 2000)
;    (send *ri* :wait-interpolation)
    ;; (prompt "Hit enter to move hand up...")
;    (send *robot* arm :inverse-kinematics (send (send *robot* arm :end-coords :copy-worldcoords) :translate #f(0 0 200) :world))
;    (moverobot 2000)
;    (send *ri* :wait-interpolation)
)

;; setup
(defun setup ()
    (task2arena)
    (objects (list *robot* *task2arena*))
    (if (boundp '*irtviewer*)
            (send *irtviewer* :draw-objects))
    )

(defun lift-arm-up ()
    (format t "Entered lift-arm-up.~%")
    (setq arm-start-pose (make-coords :pos #f(370 -470 1055))) ;; BUG IN EUSLISP!!!! setq fails to set the :pos field of the new coordinates object 
    (send arm-start-pose :replace-pos #f(370 -470 1055)) ;; BUG IN EUSLISP!!!! need to use replace-pos to directly change the :pos field
    (format t "arm-start-pose ~a~%" arm-start-pose)
;    (cond
 ;    ((equal wrenchSize 15) (send arm-start-pose :translate #f(50 -90 506)))
  ;   ((equal wrenchSize 17) (send arm-start-pose :translate #f(50 -90 526)))
   ;  )
    (format t "arm-start-pose ~a~%" arm-start-pose)
    ;(send *robot* :reset-pose)
    (send *robot* :head :neck-y :joint-angle -9)
    (send *robot* arm :inverse-kinematics arm-start-pose)
    (if (boundp `*irtviewer*)
            (send *irtviewer* :draw-objects))
    (moverobot)
)


(defun move-arm-to-start-pose ()
    (format t "Entered move-arm-to-start-pose.~%")
    (setq arm-start-pose (make-coords :pos #f(417 -272 607))) ;; BUG IN EUSLISP!!!! setq fails to set the :pos field of the new coordinates object 
    (send arm-start-pose :replace-pos #f(417 -272 607)) ;; BUG IN EUSLISP!!!! need to use replace-pos to directly change the :pos field
    (cond
     ((equal wrenchSize 15) (send arm-start-pose :translate #f(80 -60 506)))
     ((equal wrenchSize 17) (send arm-start-pose :translate #f(80 -60 526)))
     )
    (format t "arm-start-pose ~a~%" arm-start-pose)
    (send *robot* arm :inverse-kinematics arm-start-pose)
    (if (boundp `*irtviewer*)
            (send *irtviewer* :draw-objects))
    (moverobot)

    (send *ri* :wait-interpolation)
    (send arm-start-pose :orient 0 #f(1 0 0) :world)
    (cond
     ((equal wrenchSize 15) (send arm-start-pose :translate #f(0 60 0)))
     ((equal wrenchSize 17) (send arm-start-pose :translate (float-vector 0 60 0)))
     )
    (send *robot* arm :inverse-kinematics arm-start-pose)
    (if (boundp `*irtviewer*)
            (send *irtviewer* :draw-objects))
    (moverobot)
    (send *ri* :wait-interpolation)
    (send arm-start-pose :translate #f(50 0 0))
    (send *robot* arm :inverse-kinematics arm-start-pose)
    (if (boundp `*irtviewer*)
            (send *irtviewer* :draw-objects))
    (moverobot)
    (send *ri* :wait-interpolation)
    )

;; move the hand directly above the detected shaft pos 
(defun align-wrench-with-shaft ()
    (print "entered align-wrench-with-shaft")
    (setq time_now (instance ros::time :now))
;    (if (send *tfl* :wait-for-transform "/rarm_end_coords" "valve" time_now 30)
    (if (send *tfl* :wait-for-transform "/rarm_end_coords" "valve" (instance ros::time :init) 30)
;    (if (send *tfl* :wait-for-transform "/rarm_end_coords" "cluster_decomposer_finaloutput00" time_now 20)
;    (if (send *tfl* :wait-for-transform "/rarm_end_coords" "dummy_shaft" time_now 10)
       (progn
         (print "Found shaft tf.")
         (setq hand-to-shaft-coords (send *tfl* :lookup-transform "/rarm_end_coords" "valve" (instance ros::time :init)))
         (setq shaft-to-hand-coords (send *tfl* :lookup-transform "valve" "/rarm_end_coords" (instance ros::time :init)))
         (setq base-link-to-shaft-coords (send *tfl* :lookup-transform "base_link" "valve" (instance ros::time :init)))
         (format t "hand-to-shaft-coords ~a~%" hand-to-shaft-coords)
         (format t "base-link-to-shaft-coords ~a~%" base-link-to-shaft-coords)
         (send *ri* :move-waist #f(150 0 -50) :world 7000)
;         (setq hand-to-shaft-coords (send *tfl* :lookup-transform "/rarm_end_coords" "cluster_decomposer_finaloutput00" time_now))
;         (setq hand-to-shaft-coords (send *tfl* :lookup-transform "/rarm_end_coords" "dummy_shaft" (instance ros::time :init)))
         (setq shaft-world-coords (send (send *robot* :rarm :end-coords :copy-worldcoords) :translate (send hand-to-shaft-coords :pos)))
         (send base-link-to-shaft-coords :translate #f(-300 0 -200) :world)
         (send base-link-to-shaft-coords :translate #f(-54 0 20) :world) ;; account for offset in recognition
         ;; (send shaft-world-coords :rotate (send (send *robot* :rarm :end-coords :copy-worldcoords) :rot))  ;; use hand orientation

         ;; (setq arm-above-shaft-pos (send (send shaft-world-coords :copy-coords) :translate (float-vector 0 0 (+ wrenchLength 20))))
         (setq arm-above-shaft-pos (send (send base-link-to-shaft-coords :copy-coords) :translate (float-vector 0 0 (+ wrenchLength 40))))

         (send arm-above-shaft-pos :replace-rot #2f((0 1 0) (-1 0 0) (0 0 1)))
         (format t "arm-above-shaft-pos ~a~%" arm-above-shaft-pos)
         (unless (send *robot* arm :inverse-kinematics arm-above-shaft-pos)
             (return-from align-wrench-with-shaft))


         (if (boundp `*irtviewer*)
             (send *irtviewer* :draw-objects))
         (moverobot)
         (unix::sleep 7) ;; added
         ;;(send *ri* :wait-interpolation)
         )
      (progn ;; else if shaft tf not found 
        (print "No shaft detected. Wait for shaft tf failed."))
       )
    )

(defun align-wrench-with-shaft2 ()
    (print "entered align-wrench-with-shaft")
    (setq time_now (instance ros::time :now))
;    (if (send *tfl* :wait-for-transform "/rarm_end_coords" "valve" time_now 30)
    (if (send *tfl* :wait-for-transform "/rarm_end_coords" "valve" (instance ros::time :init) 30)
;    (if (send *tfl* :wait-for-transform "/rarm_end_coords" "cluster_decomposer_finaloutput00" time_now 20)
;    (if (send *tfl* :wait-for-transform "/rarm_end_coords" "dummy_shaft" time_now 10)
       (progn
         (print "Found shaft tf.")
         ;; (shaftDetectionPose)
         (unix:sleep 5) ;; wait for shaft tf to stablize
         ;; (setq hand-to-shaft-coords (send *tfl* :lookup-transform "/rarm_end_coords" "valve" (instance ros::time :init)))
         (setq shaft-to-hand-coords (send *tfl* :lookup-transform "valve" "/rarm_end_coords"  (instance ros::time :init)))
         ;; (format t "hand-to-shaft-coords ~a~%" hand-to-shaft-coords)
         (format t "shaft-to-hand-coords ~a~%" shaft-to-hand-coords)
;         (setq hand-to-shaft-coords (send *tfl* :lookup-transform "/rarm_end_coords" "cluster_decomposer_finaloutput00" time_now))
;         (setq hand-to-shaft-coords (send *tfl* :lookup-transform "/rarm_end_coords" "dummy_shaft" (instance ros::time :init)))
         ;; (setq shaft-world-coords (send (send *robot* :rarm :end-coords :copy-worldcoords) :translate (send hand-to-shaft-coords :pos)))
         ;; (send shaft-world-coords :rotate (send (send *robot* :rarm :end-coords :copy-worldcoords) :rot))  ;; use hand orientation
         ;; (setq arm-above-shaft-pos (send (send shaft-world-coords :copy-coords) :translate (float-vector 0 0 (+ wrenchLength 20))))

         (setq shaft-to-hand-pos (send shaft-to-hand-coords :pos))
         
         (send hand-target-coords (send :translate (send *robot* :rarm :end-coords :copy-worldcoords)
                                        (float-vector
                                         (- (elt shaft-to-hand-pos 1))
                                         (elt shaft-to-hand-pos 0)
                                         (+ (- (elt shaft-to-hand-pos 2)) wrenchlength 30)
                                         )
                                        :world)
               ) 
         (unless (send *robot* arm :inverse-kinematics arm-above-shaft-pos)
             (return-from align-wrench-with-shaft nil))
               
               
         (if (boundp `*irtviewer*)
             (send *irtviewer* :draw-objects))
         (moverobot)
         (send *ri* :wait-interpolation)
         )
      (progn ;; else if shaft tf not found 
        (print "No shaft detected. Wait for shaft tf failed."))
       )
    )

;; confirm that the chaft has been fitted to the wrench head 
;; by moving the dynamixel motor left and right and check for force
(defun confirm-shaft-center-found ()

  (print "in confirm-shaft-center-found")
  (slide-to-fit-move-arm #f(0 0 -5)) ;; move down a bit to press harder on to shaft first
  
  (setq gripper-err-thres 0.5)
  (setq start-gripper-pos (get-gripper-pos))

  ;; turn right 
  (gripper-servo-pos (- start-gripper-pos 10) 200)
  (unix::sleep 1)
  (format t "gripper err moving right ~a~%" (get-gripper-err))
  (setq detected-force-right (> (abs (get-gripper-err)) gripper-err-thres))

  ;; turn left 
  (gripper-servo-pos (+ start-gripper-pos 10) 200)
  (unix::sleep 1)
  (format t "gripper err moving left ~a~%" (get-gripper-err))
  (setq detected-force-left (> (abs (get-gripper-err)) gripper-err-thres))

  ;; turn back to start pos
  (gripper-servo-pos start-gripper-pos 200)
  (unix::sleep 1)

  (if (and detected-force-right detected-force-left)
      (return-from confirm-shaft-center-found 0))
  (if (and detected-force-right (not detected-force-left))
      (return-from confirm-shaft-center-found 1))
  (if (and detected-force-left (not detected-force-right))
      (return-from confirm-shaft-center-found -1))
  (return-from confirm-shaft-center-found nil)
  )

(defun get-gripper-pos ()
    (elt (send *ri* :actual-vector) 5)
    )

(defun get-gripper-err ()
    (-
     (elt (send *robot* :angle-vector) 5)
     (elt (send *ri* :actual-vector) 5)
     )
    )

(defun move-arm-to-start-pose2 ()
    (resetpose)
    (setq arm-start-pose (send *robot* arm :end-coords :copy-worldcoords))
    (send arm-start-pose :orient 0 #f(1 0 0) :world)
    (send arm-start-pose :translate #f(130 60 200))
    (send *robot* arm :inverse-kinematics arm-start-pose)
    (if (boundp `*irtviewer*)
            (send *irtviewer* :draw-objects))
    (moverobot)
    (send *ri* :wait-interpolation)
    )

;; IK for moving arm smoothly in cartesian space.
;; This assumes that shoulder-y is at -90 deg, and use the lifter to
;; move in the y direction, while using the arm to move only in the
;; z-x directions, since the arm does not have enough DOFs to move smoothly
(defun slide-to-fit-move-arm (delta)
    (format t "delta ~a~%" delta)
    (if (and (eq (elt delta 0) 0.0) (eq (elt delta 1) 0.0)) ;; if moving in approach direction
            (progn ;; moving in approach direction, can move faster
                (format t "moving in approach direction, can move faster~%")
                (format t "~%")
                (format t "~%")
                (setq slide-to-fit-moverobot-time 300)
                (setq target-pose (send *robot* arm :end-coords :copy-worldcoords))
                (send target-pose :translate delta)
                (send *robot* arm :inverse-kinematics target-pose)
                (moverobot slide-to-fit-moverobot-time)
                ;; (send *ri* :wait-interpolation)
                (unix:usleep (+ (* slide-to-fit-moverobot-time 1000) 50000))
                )
        
            (progn
                (format t "moving in tangent direction, can move slower~%")
                (format t "~%")
                (format t "~%")
                (setq slide-to-fit-moverobot-time 300)
                (setq target-pose (send *robot* arm :end-coords :copy-worldcoords))
                (send target-pose :translate delta)
                (send *robot* arm :inverse-kinematics target-pose)
                (moverobot slide-to-fit-moverobot-time)
                ;; (send *ri* :wait-interpolation)
                (unix:usleep (+ (* slide-to-fit-moverobot-time 1000) 500000))
                )
            )
        )
        
;;(setup)

(setq arm :rarm)
(setq debug t)

(setq *real* t)

(get-force-offset)
