#!/usr/bin/env roseus

(load "package://aero_utils/euslisp/start-roseus-aero-arm-single-wrist.l")

(get-force-offset)
(defun monitorForce ()
    (while t
        (setq forceVec (get-offset-force-vector))
        (format t "x: ~10,2f y: ~10,2f z: ~10,2f~%"
                (elt forceVec 0)
                (elt forceVec 1)
                (elt forceVec 2)
                )
        )
    )

(monitorForce)
