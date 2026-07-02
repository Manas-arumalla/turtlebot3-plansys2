;; The original washroom scenario, upgraded: the waypoint graph matches
;; washbot_control/config/world.yaml so this exact problem can be executed
;; end-to-end in simulation. The loop through basin/shower/commode gives
;; the planner a real routing choice — and gives replanning an alternative
;; route when a passage is blocked.

(define (problem washroom_small)
  (:domain washbot)

  (:objects
    washbot - robot
    dock hall basin commode shower - location
  )

  (:init
    (robot_at washbot dock)

    (connected dock hall)   (connected hall dock)
    (connected hall basin)  (connected basin hall)
    (connected hall commode) (connected commode hall)
    (connected basin shower) (connected shower basin)
    (connected commode shower) (connected shower commode)

    (dirty basin)
    (dirty commode)
  )

  (:goal (and
    (cleaned basin)
    (cleaned commode)
  ))
)
