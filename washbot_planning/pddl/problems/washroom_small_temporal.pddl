;; Temporal version of the washroom scenario: durations, battery drain and a
;; charger at the dock. The low starting battery forces the planner to
;; schedule a recharge before it can finish both fixtures.
;; Solve with POPF: it requires :durative-actions and :fluents.

(define (problem washroom_small_temporal)
  (:domain washbot_temporal)

  (:objects
    washbot - robot
    dock hall basin commode shower - location
  )

  (:init
    (robot_at washbot dock)
    (charger_at dock)

    (connected dock hall)   (connected hall dock)
    (connected hall basin)  (connected basin hall)
    (connected hall commode) (connected commode hall)
    (connected basin shower) (connected shower basin)
    (connected commode shower) (connected shower commode)

    (dirty basin)
    (deep_dirty commode)

    (= (battery-level washbot) 18.0)

    (= (travel-time dock hall) 8.0)    (= (travel-time hall dock) 8.0)
    (= (travel-time hall basin) 6.0)   (= (travel-time basin hall) 6.0)
    (= (travel-time hall commode) 7.0) (= (travel-time commode hall) 7.0)
    (= (travel-time basin shower) 5.0) (= (travel-time shower basin) 5.0)
    (= (travel-time commode shower) 5.0) (= (travel-time shower commode) 5.0)

    (= (travel-drain dock hall) 2.0)    (= (travel-drain hall dock) 2.0)
    (= (travel-drain hall basin) 2.0)   (= (travel-drain basin hall) 2.0)
    (= (travel-drain hall commode) 2.0) (= (travel-drain commode hall) 2.0)
    (= (travel-drain basin shower) 2.0) (= (travel-drain shower basin) 2.0)
    (= (travel-drain commode shower) 2.0) (= (travel-drain shower commode) 2.0)

    (= (clean-time basin) 15.0)
    (= (deep-clean-time commode) 40.0)
    (= (clean-drain basin) 5.0)
    (= (clean-drain commode) 8.0)
  )

  (:goal (and
    (cleaned basin)
    (cleaned commode)
  ))

  (:metric minimize (total-time))
)
