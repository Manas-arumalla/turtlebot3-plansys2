;; WashBot STRIPS domain.
;;
;; The minimal classical model of a cleaning mission: a robot moves along a
;; connectivity graph and cleans dirty locations. Used for fast planning and
;; for replanning after execution failures. Timing and battery are modelled
;; in domain_temporal.pddl.

(define (domain washbot)
  (:requirements :strips :typing)

  (:types
    robot location
  )

  (:predicates
    (robot_at ?r - robot ?l - location)
    (connected ?from - location ?to - location)
    (dirty ?l - location)
    (cleaned ?l - location)
  )

  ;; Move along one edge of the connectivity graph. Edges are directed;
  ;; the problem file declares both directions for two-way passages, which
  ;; lets the executor block a single direction after a navigation failure.
  (:action move
    :parameters (?r - robot ?from - location ?to - location)
    :precondition (and
      (robot_at ?r ?from)
      (connected ?from ?to)
    )
    :effect (and
      (not (robot_at ?r ?from))
      (robot_at ?r ?to)
    )
  )

  ;; Clean the fixture at the robot's current location. Requiring (dirty ?l)
  ;; keeps the planner from scheduling pointless re-cleans.
  (:action clean
    :parameters (?r - robot ?l - location)
    :precondition (and
      (robot_at ?r ?l)
      (dirty ?l)
    )
    :effect (and
      (not (dirty ?l))
      (cleaned ?l)
    )
  )
)
