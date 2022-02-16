(define (domain robot) 
   (:requirements :strips) 
   (:predicates (position ?x) (neighborTop ?x ?y) (neighborRight ?x ?y) (neighborLeft ?x ?y) (neighborBottom ?x ?y) (clear ?x)) 

   (:action moveStraight 
       :parameters (?from ?to) 
       :precondition (and (position ?from) (neighborTop ?from ?to) (clear ?to)) 
       :effect (position ?to))
	   
	(:action moveRight 
       :parameters (?from ?to) 
       :precondition (and (position ?from) (neighborRight ?from ?to) (clear ?to)) 
       :effect (position ?to))
	   
	(:action moveLeft 
       :parameters (?from ?to) 
       :precondition (and (position ?from) (neighborLeft ?from ?to) (clear ?to)) 
       :effect (position ?to))
	   
	(:action moveBack 
       :parameters (?from ?to) 
       :precondition (and (position ?from) (neighborBottom ?from ?to) (clear ?to)) 
       :effect (position ?to) 
)
)

