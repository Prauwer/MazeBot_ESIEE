	;; Evalbot (Cortex M3 de Texas Instrument)
; programme - GROUPE NUMERO 7 MazeBot // Pilotage d'un robot qui effectue un chemin guidé puis restitue les virages effectués


		AREA    |.text|, CODE, READONLY

; This register controls the clock gating logic in normal Run mode
SYSCTL_PERIPH_GPIO EQU		0x400FE108	; SYSCTL_RCGC2_R (p291 datasheet de lm3s9b92.pdf)

; The GPIODATA register is the data register
GPIO_PORTD_BASE		EQU		0x40007000	; GPIO Port D (APB) base: 0x4000.7000 (p416 datasheet de lm3s9B92.pdf)
	
; The GPIODATA register is the data register
GPIO_PORTE_BASE		EQU		0x40024000	; GPIO Port E (APB) base: 0x4000.7000 (p416 datasheet de lm3s9B92.pdf)
	
; The GPIODATA register is the data register
GPIO_PORTF_BASE		EQU		0x40025000	; GPIO Port F (APB) base: 0x4002.5000 (p416 datasheet de lm3s9B92.pdf)

; configure the corresponding pin to be an output
; all GPIO pins are inputs by default
GPIO_O_DIR   		EQU 	0x00000400  ; GPIO Direction (p417 datasheet de lm3s9B92.pdf)

; The GPIODR2R register is the 2-mA drive control register
; By default, all GPIO pins have 2-mA drive.
GPIO_O_DR2R   		EQU 	0x00000500  ; GPIO 2-mA Drive Select (p428 datasheet de lm3s9B92.pdf)

; Digital enable register
; To use the pin as a digital input or output, the corresponding GPIODEN bit must be set.
GPIO_O_DEN  		EQU 	0x0000051C  ; GPIO Digital Enable (p437 datasheet de lm3s9B92.pdf)

; Pul_up
GPIO_I_PUR   		EQU 	0x00000510  ; GPIO Pull-Up (p432 datasheet de lm3s9B92.pdf)

; Broches select
BROCHE0				EQU		0x01		; Bumper_R sur broche 0
BROCHE1				EQU		0x02		; Bumper_L sur broche 1
BROCHE0_1			EQU		0x03		; Bumper_L&R sur broche 0 et 1

BROCHE4				EQU 	0x10		; led1
BROCHE5				EQU		0x20		; led2
BROCHE4_5			EQU		0x30		; led1 & led2 sur broche 4 et 5

BROCHE6				EQU 	0x40		; bouton poussoir 1
BROCHE7				EQU		0x80		; bouton poussoir 2
BROCHE6_7			EQU		0xC0		; boutons poussoirs 1 et 2
		
			
		ENTRY
		EXPORT	__main
		

		;; The IMPORT command specifies that a symbol is defined in a shared object at runtime.
		IMPORT	MOTEUR_INIT					; initialise les moteurs (configure les pwms + GPIO)
		
		IMPORT	MOTEUR_DROIT_ON				; activer le moteur droit
		IMPORT  MOTEUR_DROIT_OFF			; déactiver le moteur droit
		IMPORT  MOTEUR_DROIT_AVANT			; moteur droit tourne vers l'avant
		IMPORT  MOTEUR_DROIT_ARRIERE		; moteur droit tourne vers l'arrière
		IMPORT  MOTEUR_DROIT_INVERSE		; inverse le sens de rotation du moteur droit
		
		IMPORT	MOTEUR_GAUCHE_ON			; activer le moteur gauche
		IMPORT  MOTEUR_GAUCHE_OFF			; déactiver le moteur gauche
		IMPORT  MOTEUR_GAUCHE_AVANT			; moteur gauche tourne vers l'avant
		IMPORT  MOTEUR_GAUCHE_ARRIERE		; moteur gauche tourne vers l'arrière
		IMPORT  MOTEUR_GAUCHE_INVERSE		; inverse le sens de rotation du moteur gauche



__main	


		; ;; Enable the Port F & D peripheral clock 		(p291 datasheet de lm3s9B96.pdf)
		; ;;									
		ldr r6, = SYSCTL_PERIPH_GPIO  			;; RCGC2
        mov r0, #0x00000038  					;; Enable clock sur GPIO D, E, et F où sont branchés les leds (0x28 == 0b111000)
		; ;;														 									      (GPIO::FEDCBA)
        str r0, [r6]
		
		; ;; "There must be a delay of 3 system clocks before any GPIO reg. access  (p413 datasheet de lm3s9B92.pdf)
		nop	   									;; tres tres important....
		nop	   
		nop	   									;; pas necessaire en simu ou en debbug step by step...
	
		;^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^CONFIGURATION LEDs

        ldr r12, = GPIO_PORTF_BASE+GPIO_O_DIR    ;; 1 Pin du portF en sortie (broche 4 : 00010000)
        ldr r0, = BROCHE4_5 	
        str r0, [r12]
		
		ldr r12, = GPIO_PORTF_BASE+GPIO_O_DEN	;; Enable Digital Function 
        ldr r0, = BROCHE4_5		
        str r0, [r12]
		
		ldr r12, = GPIO_PORTF_BASE+GPIO_O_DR2R	;; Choix de l'intensité de sortie (2mA)
        ldr r0, = BROCHE4_5			
        str r0, [r12]
		
		;; NE PAS TOUCHER R12
		ldr r12, = GPIO_PORTF_BASE + (BROCHE4_5<<2)  ;; @data Register = @base + (mask<<2) ==> LED1
		
		;vvvvvvvvvvvvvvvvvvvvvvv Fin configuration LEDs 
		
		;^^^^^^^^^^^^^^^^^^^^^^^^ MEMO LEDs
		
		;;mov r0, #0x000       	;; pour eteindre LED
		;;mov r1, #BROCHE4		;; Allume LED 1 port F broche 4 : 00010000 (DROITE)
		;;mov r2, #BROCHE5		;; Allume LED 2 port F broche 4 : 00100000 (GAUCHE)
		;;mov r3, #BROCHE4_5	;; Allume LED1&2 portF broche 4&5 : 00110000
		
		;;str r0, [r12]  		;; Eteint LED1&2 portF broche 4&5 : 00000000 (contenu de r0)
		;;str r1, [r12]  		;; Allume LED1 portF broche 4&5 : 00010000 (contenu de r1)
		;;str r2, [r12]  		;; Allume LED2 portF broche 4&5 : 00100000 (contenu de r2)
		;;str r3, [r12]  		;; Allume LED1&2 portF broche 4&5 : 00110000 (contenu de r3)

		;vvvvvvvvvvvvvvvvvvvvvvv FIN MEMO LEDs


		;^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^CONFIGURATION Switchers

		ldr r11, = GPIO_PORTD_BASE+GPIO_I_PUR	;; Pul_up 
        ldr r0, = BROCHE6_7	
        str r0, [r11]
		
		ldr r11, = GPIO_PORTD_BASE+GPIO_O_DEN	;; Enable Digital Function 
        ldr r0, = BROCHE6_7	
        str r0, [r11]     
		
		;; NE PAS TOUCHER R11
		ldr r11, = GPIO_PORTD_BASE + (BROCHE6_7<<2)  ;; @data Register = @base + (mask<<2) ==> Switchers
		
		;vvvvvvvvvvvvvvvvvvvvvvvFin configuration Switcher 
		;^^^^^^^^^^^^^^^^^^^^^^^^ MEMO Switchers
		
		;ldr r0,[r11] 		; Lire les données des switchers
		;
		;CMP r0,#0x00 		; Les DEUX Switchers port D broche 6 et 7 sont appuyés
		;CMP r0,#0x80		; Le Switcher 1 port D broche 6 est appuyé
		;CMP r0,#0x40		; Le Switcher 2 port D broche 7 est appuyé
		
		;vvvvvvvvvvvvvvvvvvvvvvv FIN MEMO Switchers
		
		
		;^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^CONFIGURATION Bumpers

		ldr r10, = GPIO_PORTE_BASE+GPIO_I_PUR	;; Pul_up 
        ldr r0, = BROCHE0_1	
        str r0, [r10]
		
		ldr r10, = GPIO_PORTE_BASE+GPIO_O_DEN	;; Enable Digital Function 
        ldr r0, = BROCHE0_1
        str r0, [r10]     
		
		;; NE PAS TOUCHER R10
		ldr r10, = GPIO_PORTE_BASE + (BROCHE0_1<<2)  ;; @data Register = @base + (mask<<2) ==> Bumpers
		
		;vvvvvvvvvvvvvvvvvvvvvvvFin configuration Bumpers 
		;^^^^^^^^^^^^^^^^^^^^^^^^ MEMO Bumers
		
		;ldr r0,[r10] 		; Lire les données des switchers
		;
		;CMP r0,#0x00 		; Les DEUX Bumpers port E broche 0 et 1 sont appuyés
		;CMP r0,#0x02		; Le Bumper R port E broche 0 est appuyé
		;CMP r0,#0x01		; Le Bumper L port E broche 1 est appuyé
		
		;vvvvvvvvvvvvvvvvvvvvvvv FIN MEMO Bumers
		
		
		;; DEBUT DU PROGRAMME ;;
		
		;Initialisation du compteur de tournants à 0
		LDR	R0, =compteur
		mov r1, 0
		str r1, [r0]
		
		
		; Configure les PWM + GPIO
		BL	MOTEUR_INIT
		
		;; Etat éteint du robot
OffState
		BL	MOTEUR_DROIT_OFF ; Eteindre les moteurs
		BL	MOTEUR_GAUCHE_OFF
		mov r3, #0x00; ; Eteindre les lumières
		str r3, [r12];
		BL WAIT_BOUTON
		
		;; Vérification des input en état OFF
loopOff
		; Le switcher 1 est appuyé
		ldr r0,[r11]
		CMP r0,#0x80
		BEQ OnState
		
		; Le switcher 2 est appuyé
		CMP r0,#0x40
		BEQ ShowState
		
		B loopOff
		
		;; Etat allumé du robot
OnState
		; Allumer les deux lumières
		mov r3, #BROCHE4_5
		str r3, [r12]	   		   
		
		; Activer les deux moteurs droit et gauche
		BL	MOTEUR_DROIT_ON
		BL	MOTEUR_GAUCHE_ON

		; Evalbot avance droit devant
		BL	MOTEUR_DROIT_AVANT	   
		BL	MOTEUR_GAUCHE_AVANT
		BL WAIT_BOUTON
		
		;; Vérification des input en état ON
loopOn
		; Le bumper L est appuyé
		ldr r0, [r10]
		CMP r0, #0x01
		BEQ TurnRight
		
		; Le bumper R est appuyé
		ldr r0, [r10]
		CMP r0, #0x02
		BEQ TurnLeft
		
		; Le switch 1 est appuyé
		ldr r0,[r11]
		CMP r0,#0x80
		BEQ OffState

		b	loopOn
		
		;; Etat "tourne à droite" du robot
TurnRight
		; Stockage mémoire
		LDR R0, =directions
		LDR	R3, =compteur
		LDR r1, [r0]
		LDR r4, [r3]

		; directions
		mov r2, #0x2 ; On met la valeur 0x2 pour signifier le tournant à droite
		str r2, [r0,r4, LSL #2] ; On le stocke dans notre variable directions avec un offset équivalent au compteur de tours (LSL #2 permet de décaler de 4 octets = 2 bits de décalage à gauche)
				
		; incrémentation du compteur de tournants
		add r4, r4, #1
		str r4, [r3]


		; Recule
		BL MOTEUR_GAUCHE_ARRIERE
		BL MOTEUR_DROIT_ARRIERE
		BL WAIT_RECUL

		; Tourne à droite
		BL MOTEUR_GAUCHE_AVANT
		BL MOTEUR_DROIT_ARRIERE
		BL WAIT_TURN

		b	OnState
		
		;; Etat "tourne à gauche" du robot
TurnLeft
		; Stockage mémoire
		LDR R0, =directions
		LDR	R3, =compteur
		LDR r1, [r0]
		LDR r4, [r3]

		; directions
		mov r2, #0x1 ; On met la valeur 0x1 pour signifier le tournant à gauche
		str r2, [r0,r4, LSL #2] ; On le stocke dans notre variable directions avec un offset équivalent au compteur de tours (LSL #2 permet de décaler de 4 octets = 2 bits de décalage à gauche)

		; incrémentation du compteur de tournants	
		add r4, r4, #1
		str r4, [r3]
		
		; Recule
		BL MOTEUR_GAUCHE_ARRIERE
		BL MOTEUR_DROIT_ARRIERE
		BL WAIT_RECUL
		
		; Tourne à gauche
		BL MOTEUR_GAUCHE_ARRIERE
		BL MOTEUR_DROIT_AVANT
		BL WAIT_TURN
		
		b	OnState

		;; Restitution des données
ShowState

		MOV r0, #0        		 ; Initialisation d'un itérateur à 0
		LDR r6, =compteur  		 ; Valeur N max pour la boucle
		LDR r2, =directions		 ; tableau des directions prises par le robot
ShowLoop		                 
		LDR r3, [r6]             
								 
		CMP r0, r3	         	 ; Comparaison de l'itéation avec N
		BGE end_ShowLoop   		 ; Fin de la boucle si r0 >= r3
								 
		LDR r8, [r2, r0, LSL #2] ; Sinon, on lit la donnée dans le tableau équivalente à itérateur * 4
								 
		CMP r8, #0x1			 ; Vérifier si le robot est parti à gauche
		BEQ LeftLed				 ; Allumer la Led de gauche
								 
		CMP r8, #0x2			 ; Vérifier si le robot est parti à droite
		BEQ RightLed             ; Allumer la Led de droite
								 
ContinueLoop		             ; Après avoir clignoté, on retourne ici  
								 
		ADDS r0, r0, #1	   		 ; Incrémentation de l'itérateur
		B ShowLoop       		 ; Retourner au début de la boucle
								 
end_ShowLoop                     
		mov r0, #0				 ; Remise du compteur à 0
		str r0, [r6]
		B OffState
	
		;; Clignotement de la Led gauche
LeftLed
		mov r7, #BROCHE5
		str r7, [r12]
		BL WAIT_BLINK
		BL WAIT_BLINK
		mov r7, 0x000
		str r7, [r12]
		BL WAIT_BLINK
		
		B ContinueLoop
		
		;; Clignotement de la Led droite
RightLed
		mov r7, #BROCHE4
		str r7, [r12]
		BL WAIT_BLINK
		BL WAIT_BLINK
		mov r7, 0x000
		str r7, [r12]
		BL WAIT_BLINK
		
		B ContinueLoop

		;; Boucles d'attente ;;
		;; REFERENCE DE TEMPS POUR LES BOUCLES D'ATTENTE : 3 secondes = 0xEC4EC4

		;; Boucle d'attente pour les boutons
WAIT_BOUTON	ldr r1, =0xFFFFF ; 0.203 secondes
waitbouton	subs r1, #1
        bne waitbouton

		BX LR	; fin boucle


		;; Boucle d'attente pour le demi tour avant tourner
WAIT_RECUL	ldr r1, =0x4EC4EC ; 1 secondes
waitrecul	subs r1, #1
        bne waitrecul

		BX LR	; fin boucle

		;; Boucle d'attente pour le tournant
WAIT_TURN	ldr r1, =0x4CAFFE ; 0.94 secondes
waitturn	subs r1, #1
        bne waitturn

		BX LR	; fin boucle
		
		;; Boucle d'attente pour le clignotement
WAIT_BLINK	ldr r1, =0x1F86EE ; 0.4 secondes
waitblink	subs r1, #1
        bne waitblink

		BX LR	; fin boucle

		;; FIN DE ZONE POUR LES TEMPS D'ATTENTE
		
		NOP
		;; FIN DU PROGRAMME
		
		;; Variables de stockage des tournants ;;
		AREA variables, DATA, READWRITE
directions	SPACE	400
compteur	SPACE	4

		
        END
