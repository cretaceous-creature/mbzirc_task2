(load "package://aero_utils/euslisp/aero-arm-single-wrist-utils.l")
(load "package://mbzirc_task2_control/euslisp/move-base-util.l")


(setq isMoveRIBase t)
(setq cartWidth 0.5) ;; units in m !!!

(defun prompt (msg)
    (if debug
            (progn
                (print msg)
                (read-char)))
    )


(defun getSingleXDist (index) 
    (* (elt ranges index) (cos  (+ angle_min (* index angle_increment))))
    )

(defun getSingleYDist (index) 
    (* (elt ranges index) (- (sin  (+ angle_min (* index angle_increment)))))
    )

(defun getIndex ()
    )

(defun isThereObstacleWithinDist (distX)
    (ros::spin-once)
    (setq angle_limit (atan (/ (/ cartWidth 2.0) distX)))
    (setq minIndex (round (/ (- (- angle_limit) angle_min) angle_increment)))
    (setq maxIndex (round (/ (- angle_limit angle_min) angle_increment)))

    (setq index minIndex)
    (setq pointsWithinDistXcount 0.0)
    (while (< index maxIndex)
        ;; (format t "index ~f : angle ~f : xdist ~f~%" index
        ;;                                              (rad2deg (+ angle_min (* index angle_increment)))
        ;;                                              (getSingleXDist index))
        (if ( < (getSingleXDist index) distX)
                (setq pointsWithinDistXcount (+ pointsWithinDistXcount 1.0)))
        (setq index (+ index 1))
        )
    ;; (format t "% points with in distx ~f : ~f~%" distx (/ pointsWithinDistXcount (- maxIndex minIndex)))
    (/ pointsWithinDistXcount (- maxIndex minIndex)) 
    )

;; assumes the robot is between the panel and the wall 
(defun findPanelLocation (leftright)
    (ros::spin-once)
    (if (equal leftright :left)
            (progn
                (setq minIndex 0)
                (setq maxIndex (floor (/ (length ranges) 2)))
                )
        (progn
            (setq minIndex (floor (/ (length ranges) 2)))
            (setq maxIndex (length ranges))
            )
        )

    (setq panelMaxDist 5.0) ;; only take scan points that are within this distance to find panle cluster

    (setq sumX 0)
    (setq sumY 0)
    (setq index minIndex)
    (setq numPoints 0)

    (while (< index maxIndex)
      (if (< (elt ranges index) panelMaxDist)
          (progn
            (setq sumX (+ sumX (getSingleXDist index)))
            (setq sumY (+ sumY (getSingleYDist index)))
            (setq numPoints (+ numPoints 1)))
        )
      (setq index (+ index 1))
      )    

    ;; if didn't see anything within panelMaxDist increase serach range
    (while (< numPoints 10) ;; needed to prevent divide by zero exception
      (setq panelMaxDist (+ panelMaxDist 1.0))
      (if (> panelMaxDist 10.0)
          (return-from findPanelLocation nil))
      (while (< index maxIndex)
        (if (< (elt ranges index) panelMaxDist)
            (progn
              (setq sumX (+ sumX (getSingleXDist index)))
              (setq sumY (+ sumY (getSingleYDist index)))
              (setq numPoints (+ numPoints 1)))
          )
        (setq index (+ index 1))
        )
      )
    
    (setq avgX (/ sumX numPoints))
    (setq avgY (/ sumY numPoints))

    (setq panelX avgX) ;; global var to be used outside
    (setq panelY avgY) ;; global var to be used outside
    
    (format t "estimated panel x : ~f y: ~f~%" panelX panelY)
    t
    )

(defun callbackScan (msg)
    ;; (format t "in callbackScan")
    ;; (format t "size ~d~%" (length (send msg :ranges)))

    ;; (format t "range [0] ~f~%" (elt (send msg :ranges) 0))

    
    (setq angle_min (send msg :angle_min))
    (setq angle_max (send msg :angle_max))
    (setq angle_increment (send msg :angle_increment))
    (setq ranges (send msg :ranges))
 
    ;; (format t "x front  ~f~%" (getsinglexdist 360))
    (isthereobstacleWithinDist 2.0)
      
    )

(defun detect-panel-move-base (vel_x vel_y vel_theta time_msec)
    (if (not isMoveRIBase)
            (progn
                ;; (format t "isMoveRIBase is nil!!!~%")
                (return-from detect-panel-move-base nil))
        )
    (setq twist_msg (instance geometry_msgs::TwistStamped :init))
    (send twist_msg :twist :linear :x vel_x)
    (send twist_msg :twist :linear :y vel_y)
    (send twist_msg :twist :angular :z vel_theta)
    (send twist_msg :header :stamp (instance ros::time :init :nsec (* time_msec 1000 1000)))
    
    (format t "sending x y theta time ~a ~a ~a ~a~%"
            (send twist_msg :twist :linear :x)
            (send twist_msg :twist :linear :y)
            (send twist_msg :twist :angular :z)
            (send twist_msg :header :stamp))
    
    (ros::publish "move_base_simple/velocity" twist_msg)
    )

;; move forward untill seeing a wall withing xdist meters, moving at speed of xvel
(defun move-to-wall-front (xdist xvel)
    (format t "entered move-to-wall-front~%")
    (setq curr_xvel 0)
    (setq decelerationDist 0.5)
    (while (< (isThereObstacleWithinDist (+ xdist decelerationDist)) 0.5)
        (ros::spin-once)
        ;; (format t "isThereObstacleWithinDist ~f returned ~f~%" xdist isThereObstacleWithinDistResult)
        ;; (format t "moving base")

        (detect-panel-move-base curr_xvel 0 0 50)
        (if (< curr_xvel xvel)
                (setq curr_xvel (+ curr_xvel 0.005))
                      )
        )
    (while (> curr_xvel 0)
        (setq curr_xvel (- curr_xvel 0.01))
        (format t "curr_xvel ~f~%" curr_xvel)
        (detect-panel-move-base curr_xvel 0 0 50)
        )
    (detect-panel-move-base 0 0 0 50)
    ;; (format t "isThereObstacleWithinDist ~f returned ~f~%" xdist isThereObstacleWithinDistResult)
    (format t "exiting~%")
    )

(defun ransacLine ()

    ;; select 2 data points at random

    (setq iterations 30) 

        
    (setq bestInlierIndice (list))
    (setq bestOutlierIndice (list))
    (setq bestSlope 0)
    (setq bestIntercept 0)

    (setq inlierIndice (list))
    (setq outlierIndice (list))
    
    (setq iterationCount 0)
    (while (< iteration interationCount)
                                         
        (setq randIndex (random (length ranges)))
        (setq x0 (getSingleXDist randIndex))
        (setq y0 (getSingleYDist randIndex))
        (setq randIndex (random (length ranges)))
        (setq x1 (getSingleXDist randIndex))
        (setq y1 (getSingleYDist randIndex))
        
        (setq slope (/ (- y1 y0) (- x1 x0)))
        (setq intercept (- y1 (* slope x1)))

        (setq index 1)
        (setq lineErrorThres 4.0)
        ;; calculate error of each point 
        (while (< index (length ranges))
            (if (< (lineError slope intercept lineErrorThres))
                    (append inlierIndice index)
                (append outlierIndice index))
            (setq index (+ index 1)))

        ;; if the current line is better than the best line so far
        (if (> (length inlierIndices) (length bestInlierIndices))
                (progn
                    (setq bestInlierIndices inlierIndices)
                    (setq bestoutlierIndices outlierIndices)
                    (setq bestSlope slope)
                    (setq bestIntercept intercept)
                    )
            )
        )
    (format t "bestSlope ~f~%" bestSlope)
    (format t "bestIntercept ~f~%" bestIntercept)
    )

(defun lineError (slope intercept pointx pointy)
    (setq term1 (-
                 (/
                  (+ pointx (* slope pointy) (- (* slope intercept)))
                  (+ (* slope slope) 1)
                  )
                 pointx))
    (setq term2 (*
                 (slope
                  (+
                   (/
                    (+ pointx (* slope pointy) (- (* slope intercept)))
                    (+ (* slope slope) 1)
                    )
                   )
                  intercept
                  (- pointy)
                  )
                 ))
    (setq error (sqrt (* term1 term1) (* term2 term2)))
    )

(defun search-base-motion (side current-time &key (max-x-motion-time 3) (x-motion-dist 5000) (side-motion-dist 5000))
  (let (side-target)
    (if (equal (mod current-time (+ max-x-motion-time 1)) 0)
        (progn
          (if (equal side :right)
              (setq side-target (- side-motion-dist)))
          (move-base-side side-target :use-ri use-ri))
      (if (< (mod current-time (* (+ max-x-motion-time 1) 2)) (+ max-x-motion-time 1))
          (move-base x-motion-dist 0 :use-ri use-ri)
        (move-base (- x-motion-dist) 0 :use-ri use-ri)))))

(defun far-range-approach (targetDistFromWall startPanelY &key (max-side-motion-time 3) (side-motion-dist 5000) (x-motion-dist 5000))


    (setq initialSideMove 0)
   (format t " startpanely ~f~%" startpanely)
    (progn
                (if (and (>= startPanelY 0.0) ;; panel to the left
                         (<= startPanelY 3.0))
                        (progn
                            (send *ri* :go-pos 0 0 (deg2rad -90) 2000) ;; turn right
                            (unix:sleep 4)
                            (send *ri* :go-pos 3000 0 0 3000)
                            (unix:sleep 5)
                            (send *ri* :go-pos 0 0 (deg2rad 90) 2000)
                            (unix:sleep 4)
                            (setq initialSideMove -3) ;; in y direction of initial frame
                            )
                    )
                (if (and (< startPanelY 0.0) ;; panel to the right
                         (>= startPanelY -3.0))
                        (progn
                            (send *ri* :go-pos 0 0 (deg2rad 90) 2000) ;; turn left
                            (unix:sleep 4)
                            (send *ri* :go-pos 3000 0 0 3000)
                            (unix:sleep 5)
                            (send *ri* :go-pos 0 0 (deg2rad -90) 2000)
                            (unix:sleep 4)
                            (setq initialSideMove 3) ;; in y direction of initial frame
                            )
                    )
    
    
    (setq minDistFromWall 1.0)
    (if (< targetDistFromWall minDistFromWall)
            (setq targetDistFromWall minDistFromWall))
    (format t "targetDistFromWall ~f~%" targetDistFromWall)
    (setq approachVel 50)
    (move-to-wall-front targetDistFromWall approachVel)

    (send *ri* :go-pos 0 0 (deg2rad -90) 4000)
    (unix:sleep 4)
    (send *ri* :go-pos (* (- startpanelY) 1000) 0 0)
    (unix:sleep 6)
    (send *ri* :go-pos (* initialSideMove 1000) 0 0)
    (unix:sleep 6)
    


    t
    )

;;(ros::roseus "laser-panel-detector")

(ros::advertise "move_base_simple/velocity" geometry_msgs::TwistStamped) 
(ros::subscribe "/scan"
                sensor_msgs::LaserScan
                #'callbackScan 1)

;; (ros::rate 10)
;; (while (ros::ok)
    ;; (ros::spin-once)
    ;; (ros::sleep)
    ;; )
