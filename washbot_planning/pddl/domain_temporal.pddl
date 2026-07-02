;; WashBot temporal domain (PDDL 2.1).
;;
;; Adds what the STRIPS model abstracts away: action durations taken from
;; the world model, battery drain on movement and cleaning, deep-clean as a
;; separate longer action, and recharging at the dock. Requires a temporal
;; planner such as POPF (the planner PlanSys2 ships with); the bundled
;; internal planner intentionally rejects this file.

(define (domain washbot_temporal)
  (:requirements :strips :typing :durative-actions :fluents)

  (:types
    robot location
  )

  (:predicates
    (robot_at ?r - robot ?l - location)
    (connected ?from - location ?to - location)
    (dirty ?l - location)
    (deep_dirty ?l - location)
    (cleaned ?l - location)
    (charger_at ?l - location)
  )

  (:functions
    (battery-level ?r - robot)
    (travel-time ?from - location ?to - location)
    (travel-drain ?from - location ?to - location)
    (clean-time ?l - location)
    (deep-clean-time ?l - location)
    (clean-drain ?l - location)
  )

  (:durative-action move
    :parameters (?r - robot ?from - location ?to - location)
    :duration (= ?duration (travel-time ?from ?to))
    :condition (and
      (at start (robot_at ?r ?from))
      (over all (connected ?from ?to))
      (at start (>= (battery-level ?r) (travel-drain ?from ?to)))
    )
    :effect (and
      (at start (not (robot_at ?r ?from)))
      (at end (robot_at ?r ?to))
      (at end (decrease (battery-level ?r) (travel-drain ?from ?to)))
    )
  )

  (:durative-action clean
    :parameters (?r - robot ?l - location)
    :duration (= ?duration (clean-time ?l))
    :condition (and
      (over all (robot_at ?r ?l))
      (at start (dirty ?l))
      (at start (>= (battery-level ?r) (clean-drain ?l)))
    )
    :effect (and
      (at end (not (dirty ?l)))
      (at end (cleaned ?l))
      (at end (decrease (battery-level ?r) (clean-drain ?l)))
    )
  )

  (:durative-action deep_clean
    :parameters (?r - robot ?l - location)
    :duration (= ?duration (deep-clean-time ?l))
    :condition (and
      (over all (robot_at ?r ?l))
      (at start (deep_dirty ?l))
      (at start (>= (battery-level ?r) (clean-drain ?l)))
    )
    :effect (and
      (at end (not (deep_dirty ?l)))
      (at end (cleaned ?l))
      (at end (decrease (battery-level ?r) (clean-drain ?l)))
    )
  )

  (:durative-action recharge
    :parameters (?r - robot ?l - location)
    :duration (= ?duration 30)
    :condition (and
      (over all (robot_at ?r ?l))
      (over all (charger_at ?l))
    )
    :effect (and
      (at end (assign (battery-level ?r) 100))
    )
  )
)
