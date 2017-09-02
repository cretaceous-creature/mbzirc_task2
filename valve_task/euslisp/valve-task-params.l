(setq wrenchSize 19)

(cond
 ((equal wrenchSize 15) (setq wrenchLength 165))
 ((equal wrenchSize 16) (setq wrenchLength 175))
 ((equal wrenchSize 17) (setq wrenchLength 183))
 ((equal wrenchSize 18) (setq wrenchLength 192))
 ((equal wrenchSize 19) (setq wrenchLength 201))
 ;; ((equal wrenchSize 22) (setq wrenchLength 265)) ;; longer version
 ((equal wrenchSize 22) (setq wrenchLength 226)) ;; longer version
 ;; ((equal wrenchSize 22) (setq wrenchLength 227)) ;; shorter version
 ((equal wrenchSize 24) (setq wrenchLength 245))
 )   
