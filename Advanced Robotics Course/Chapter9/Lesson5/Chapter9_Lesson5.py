# PDDL-like schemas (conceptual, not executable)
(:action move
 :parameters (?r - robot ?from - conf ?to - conf)
 :precondition (and (Reachable ?r ?from ?to))
 :effect (and (AtConf ?r ?to) (not (AtConf ?r ?from))))

(:action pick
 :parameters (?r - robot ?o - object ?s - surface ?g - grasp)
 :precondition (and (At ?o ?s)
                    (EmptyHand)
                    (GraspReachable ?r ?o ?s ?g))
 :effect (and (Holding ?o)
              (not (At ?o ?s))
              (not (EmptyHand))))

(:action place
 :parameters (?r - robot ?o - object ?s - surface ?g - grasp)
 :precondition (and (Holding ?o)
                    (PlaceReachable ?r ?o ?s ?g))
 :effect (and (At ?o ?s)
              (EmptyHand)
              (not (Holding ?o))))
      
