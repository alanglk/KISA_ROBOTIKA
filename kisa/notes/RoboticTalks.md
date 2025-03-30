# Playing with robots: issues and experiences
```
Andrea Bonarini 
Airlab
17 de marzo de 2025
```

A game is ruled while playing is one of the basic activities to learn, train or entertain.
"Playing with robots is the way we can bring robots at home" and it is a great HRI (Human-Robot-Interaction).

If there are rules in the play, the robot must follow them in order to play the game and there is no space for ineffective or fake interaction.
Some of the critical issues of the robots when playing with humans are: time, to be realtime responsive; fragile users, such as kids or disabled people and there are some ethical issues to consider.

General market view:
- Millions of robots are entering the toy market every year
- None of them ar used in proffesional application
- They cost less than 200€ (there is a hardware limitation that has to be accomplised)

Traditional toys vs toyrobots:
Toy robots share with traditional toys some similarities such as shapes, materials, manipulation possibilities and self pressentation, but robot toys manufacturers also has to consider the quality of movement (with the noise and sound associated to the movement) and they are limmited to the use of simple sensors and simple reactions (due to the hardware limitations). However, "tomorrow robot toys" are expected to react to implicit signals, should take initiative on the interactions, should be believable (to be more "human" like), are expected to make the right actions to maintain the attention of the human, should exploit more the phisical aspect and must maintain the low-cost.

Despite this, there are lots of types of play with current robots: copetitive, cooperative, first-person or avatar regarding to how the interaction is made; physical, where players are involved in some physical activity such as sports or cognitive, where players are involved in some cognitive task; free play, practice play, pretend play, contruction play or rule-based play attending the rules or type of task of play...

Also there are some key factors to consider in the robot human interaction:
- Safety feeling: humans mush percieve the dimensions and the speed of toy robots as safe.
- Fairness: the robot must comply with the rules and expected level of ability.
- Engagement: it should be fun to play with the toy robot so it should provide an interesting story, a good challenge which should not be too easy or too hard, it should promote the motivation of winning and a kind of reward and the control and rythm (not to feel stagnated) are also important.
- Channel of interaction: communication channels...


Some examples of the human robot interactions in games are showed in the jedI game, where a drone acts as a droid and the human must deffecd form lasser attacks (as in the star wars movies) or the Robotower, where a competence of who if the human or the robots reaches first the signaled button in a squared arena with four buttons in each of the corners.


Inclusive play is another of the hot topics in toy robots. The goal is that anyone should be able to play with robots to gain wheter skill or knowledge and have fun.
The deffinition of toy  is whatever interesting to play, so How we can trigger interest? with something new that gives exploration and configuration posibilities.
Inclusive toys today are: toys for "normal" developed children; toys adapted to be used by disabled people; toys for disables people and toys designed for all.

Many research issues are open to implement effective play activities:
- Realtime user modeling
- Online adaptations
- Interaction: pyshical interactions according to language and sentiments
- Play dynamics


# Social Media Mining or Human-Robot Interaction
```
Teresa Onorati
Interactive DEI Lab
Computer Science Department
18 de marzo de 2025
```

Han intentado juntar el modelado de datos semanticos con los robots sociales.
Intereses de investigacion: **Human-computer interation** semntic modeling semntic visualization inmersive analytics.


## Modelado y visualizacion semántica
Modelado semántico -> han desarrollado herramientas de visualizacion: coleccionaron tweets y con su herramienta se identificaban 

permiten a cualquier persona realizar clasificación de mensajes para identificar patrones en los mensajes 


Una visualizacion semantica es una cisualizacion de los datos en las que el usuario puede clasiicarlos, visualizarlos y que sea facil de comprender para los usuarios. Por ejemplo un treemap grafica una serie de conceptos y al clickar en un tema se muestran los subtemas relacionados.
Se tiene la informacion de lo que se está aplicando y desde donde (geograficamente)

Esta aplicacion se utilizó para emergencias, en protección civil, que con una herramienta de este tipo se puede navegar sobre la basta cantidad de datos.

En el caso del atentado de parís, permitió a las autoridades identificar un movimiento ciudadano en redes en las que los habitantes ofrecían refugio y permitió coordinar las ayudas.


## Visualización Inmersiva
Está motivada por el hecho de conseguir visualizaciones amplias de 360º en la que la persona se puede concentrar más en la tarea que está realizando. Contiene elementos 3d, 2d videos.. simulando un aula de estudio en la que el usuario es capaz de interactuar con el concepto que se está analizando.

Por ejemplo, con este tipo de herramientas se ha conseguido mejorar el aprendizaje en entornos de instituto.



## Proyecto Social media mining for human-robot interaction
Mejorar la interaccion entre los humanos y los robots sociales utilizando la información publicada en las redes sociales.
```
Social Networks -> Information -> Social Robots -> Interaction Capabilities
```

### Social robots 
Uno de los problemas que se pueden encontrar está muy relacionado con la comunicación que se establece entre el usuario y el robot. Por ello, es muy importante que estas interacciones sean divertidas y que enganchen al usuario para que este quiera seguir interactuando con el robot.

Los robots sociales están diseñados para poder interactuar de una forma más natural con las personas y que interactuen tanto verbalmente como de forma física.

Hay iniciativas de social robots for eldery, la idea es poder ver si la robotica puede ofrecer soluciones que puedan mantener una asistencia en casa a aquellas personas mayores. Deberían supevisar la persona: medicación, aburrimiento, llamar a su familia, llamar a emergencias...

Los robots sociales pueden ser parte de una solucion para personas que están solas o aisladas; pueden ser una ventana hacia el exterior para personas aisladas.. pueden ser una forma de comunicacion con amigos, familiares o los médicos; pueden ser una forma de aprender o entrenamiento cognitivo. A veces es más sencillo para las personas mayores interactuar con un robot que con un smartphone o una tablet.

La persona debe confiar en el robot le esta proponiendo o diciendo, por lo que es importante que en todo momento sea clara la utilidad del robot para construir esta confianza humano-robot.


**The social robot Mini**

Utiliza varios elementos para ser más expresivo. También tiene una tablet para ofrecer viás alternativas para interactuar con el robot ya que algunas personas mayores tienen limitacion del habla.

Dependiendo de la interaccion con la persona es el punto de inicio de todo el uso del robot mini. La persona le dice al robot que es lo que quiere hacer y el robot reacciona. Las habilidades principales que ofrece mini son:
    
- Emtretenimiento: Contar chistes, canciones, noticias, recomendaciones...
     
- Terapeutica: varias habilidades que estimulan habilidades cognitivas: juegos de lenguaje, ejercitar memoria, reconocer objetos...


Entre los objetivos de mini, uno de ellos es evitar que las personas mayores se cansasen del robot con dialogos repetitivos.
Por ello, se ha seleccionado redes sociales de acuardo al perfil que se tienen que adaptar

```
creating user profile -> Generacion de noticias a partir de redes sociales -> generar el dialogo
```

Muchos de los datos se extraen de tweeter:
Si es de madrid, datos de madrid, si le gusta un periodico datos de ese periodico... 

Se utuliza el texto de las noticias para dos tareas: resumir la noticia para comenzar la noticia y luego se generan dialogos de pregunta respuesta con el humano. Además, a parte de la informacion de la propia noticia tamvien se contestaba con información preprogramada en ciertos cassos.


A través de la interaccion con el robot se inician las tareas y si el usuario quiere hablar sobre noticias se inicia el proceso descrito.



Además de la comunicacion verbal, el robot tambien interactua con comunicacion no-verbal. Movimientos, reacciones con colores que reflejan el estado del ánimo.

En algunos casos, el robot simula que esta pensando "dejame pensar", "estoy en ello", "mueve los brazos" para que el usuario se quede pendiiente mientras el robot está procesando la información.


Se han realizado interacciones con personas mayores y las personas involucradas han dado criticas positivas en su mayoria. Sí que se ha comentado retrasos significativos en la interacción.

## Conclusiones

Solucionar algunos de los problemas del robot. Se podría automatizar la generacion de perfiles. Aumentar la velocidad de proocesamiento.

Como puntos positivos, se ha podidoo probar con personas y en muchos casos las personas creían que las interacciones estaban predefinidas lo que sugiere que el robot se ha adaptado adecuadamente al perfil de la persona.




# Aplicaciones en robótica inndustrial
```
Ander Iguiondo
Tekniker
19 de marzo de 2025
```

Lineas de investigacon Tekiniker Fabricacion avanzada superficies y materiales, Tic para produccion, ingeniería del produto

Ander está en TIC para producción y trabaja en el departamento de navegación autónoma. 

Un robot necesita:
- Percepción y sensores
- control y planificacion
- interaccion
- inteligencia artifical

En el departamento se enfocan en: movilidad manipulacion ineraccion y percepción.

## 1. Manipulación de objetos
La manipulacion es la capacidad que tienen los robots para manipular objetos. Necesitan tecnologías para gestionar:
- La variabilidad en los objetos: forma, peso, material...
- Limitaciones de las soluciones actuales: falta de flexibilidad y adaptabilidad; garras ad-hoc...

Las tecnologías abarcan desde como percibir los objetos, cual es la toma de decisiones y como planificar y gestionar la trayectoria para manipular el objeto con la correspondiente retroalimentación de la percepcion.


Ejemplo de aplicacion: bin-picking
El objetivo es el de manipular una caja y vaciarla de objetos que serían piezas industriales.
```
Image acquisition -> scene segmentation -> pose estimation -> grasping point identification
```

Ellos proponen un sistema de entrenamiento automático para modelos de segmentacion que está basado en anotaciones automáticas en simulación. 

La fase de simulacion para generar los datasets de entranamiento está basada en unity y contine bastantes parámetros para ajustar las escenas sintéticas a partir de modelos de CAD de las piezas.

Para ajustar los hiperparámetros utilizan una herramienta evolutiva que los ajusta automaticamente: raylib.


¿Cómo se coge el objeto?
Se establece un punto de agarre del robot. Se automatiza el proceso de seleccionar el punto de agarre de la pieza utilizando modelos de deeplearning que dado el CAD de la pieza devuelve las primiticas de la pieza para luego utilizando se emplean algoritmos para muestrear puntos de interés válidos con la geometria del robot y la pieza y finalmente se evaluan los puntos de agarre obteniiendo metricas. Para esta evaluacion en simulación se ha utilizado el simulador Mujoco. Finalmente, se ha testeado en un entorno real con un brazo robótico.

Uno de los problemas más significativos es el salto entre simulación y la realidad, por lo que es dificil saber si lo que se está probando en simulación es fiel con la realidad.

Se definieron varios marcadores para registrar el movimiento de un robot real y luego se utilizaron algoritmos metaheuristicos para ajustar ese mismo movimiento en simulación y comparar ambos.


¿Qué objeto cojo?
Se ha desarrollado un módulo de decisión basado en IA para, dada una caja con multiples objetos, se decida cuál es la mejor forma de tomar estos objetos. Para ello se ha entrenado un nodulo de decision con aprendizaje por refuerzo para crear políticas de agarre. Esto ha sido modeladoo como un modelo de deep-Q-learning.

El aprendizaje por refuerzo se suele realizar en simulación, pero en este caso, debido a la geometría compleja de las piezas se ha realizado un aprendizaje por refuerzo offline. 

Hay casos en los que se presentan obetos que nunca se han visto. Para esto se han entrenado modelos específicos.


Programación de robots mediante behavior trees. Inspirado del mundo de los videojuegos, permite reutilizar bloques y facilita la visualizacion y seguimientoo del flujo del programa.

También están trabajando en aprendizaje por demostración en los que el robot aprende a generar los árboles de comportamiento.

Fase de agarre: se suelen utilizar ventosas para formas regulares, pinzas para irregulares y magnetismo para piezas metálicas.

Tambien se han utilizado imágenes visuotactiles para evaluar el agarre de los robots


## 2. Navegación autonóma 
Teknikbot es un robot contruido en Tekniker capar de realizar nevagación en el espacio basado en mapas 2D, estaba basado en ROS y la localización se realizaba mediante distintos sensores.

Mainbot, es otro robot diseñado para realizar el mantenimiento de parques solares. Utilizaba un sistema RTK para la localización.

Coinspect es un robot para inspeccionar alas de avión. El sistema contiene una serie de marcas y mediante el sistema de visión se calcula la relacion de la posicion en tre el robot y el ala.

Greenpatrol diseñado para el tratamiento de plantas. Se utiliza deeplearning para identificar las plagas en las plantas.


## 3. Interaccion persona-robot 
Detección de gestos. Se pueden utilizar las coordenadas de los puntos de la mano como input para un modelo de clasificación e identificar el gesto realizado.

Interaccion por voz y gestos con brazos robóticos para eliminar las ambigüedades: "coge eso" es ambiguo pero con el gessto de señalar el robot sabe qué coger.

Programación de robots utilizando realizad virtual. Pensada para la enseñanza y para entornos industriales. Tiene dos modos: interaccion con un chatbot o interaccion con un gemelo digital utilizando gafas de realidad aumentada. Se muestra una trayectoria resultante y si se da el OK el sobot la ejecutta. De esta manera se permite la programación mediante el diálogo o definiendo de forma virtual el movimiento del robot.


Identificacion de la necesidad de ayuda en un entorno colaborativo con el robot. La idea es monitorizar la mirada y el comportamiento de la persona y con un proceso de etiquetadoo de escena manualmente se entrenaron varios clasificadores con el objetivo de ver si eran capaces de identificar las situaciones de ayuda.
Los resultados indican que en el proceso de montaje colaborativo sí que es posible identificar las situaciones en las que la persona necesita ayuda solo con las expresiones faciales


Proyecto Onicron: un robot con un brazo robótico para recoger conos y relizar tareas de mantenimiento en carretera.


## 4.  Robots fuera en industria 
Deteccion de humanos y puntos clave. Openpose
Se añade el detector de puntos clave al detector de proximidad del robot para que identifique si un humano entra en el area de operacion del robot y que este se detenga.



## 5. Seguridad en entornos industriales
Control de una apisonadora con deteccion de intenciones de humanos en BEV para que si entran en el area de peligro se les avise.



# Measuring Animation Quality
```
Taras Kucherenko
Electronics Arts
20 de marzo de 2025
```

## Introduction
Virtual agents: digital representation of humanoids
Virutal agents doesnt have the phisical limitations of robots.
Animation is the movement of the virtal agent

Measuring the animation quality is hard in general. This is the focus of the talk ¿How can we measure the animation quality?

Objective metrics: Every time we get the same animation, the number is the same.

Subjective metrics: Average people ranking of the animation and qget the mean.

The task of gesture generation is to generate an output animation from a speech. It is difficult because there are many different solutions, there are multiple gestures for the same speech; There are very little data for trainig and the gestures depends to each person, the state of the person if she has a good or bad day...

Also benchmarking is very weak, as everybody is doing his own thing.
For example, GENEA gesture generation challenge, has its own standarized pipeline except the models so it gives a fair gesture benchmarking.

GENEA 2022, used a dataset of 20 hours of audio and 3D motion, many participants and different speackers. People from ubisoft, huawey and other companies participated. There were 10 final challenge submisions.

People could judge each animation to evaluate it as bad, good and so on. With this the averaged the results to obtain a subjective rank of the samples. With this study, there people have different oppinions but the average is quite consistent and they usually agree on the evalutaions. All of the system were better than the baseline.

The winner of the challenge was not based on deep learning, it was a graph that conects real data smartly depending on the speech. Not all is deeplearning!!

Objective metrics: custom distances to measure the correlation between grountruth and predictions. They trained an encoder to encode the gestures (the groundtruth and the generated ones) in order to compare them with the metrics.

Almost none of the objectric metrics were not particularly correlated with the subjective ones (p-value < 0.05)


### Conclusions
Objective metrics did not correlate well with subjective ones. More works needs to be put in making good objective data to measure gesture quality.



## Missing markers reconstruction
Motion capture is a way of obtaining animation data and there are multiple ways to do that. The tradirional way is to use motion capture suits with motion actors. However this is expensive.

Also that output data has some problems such as missing markers on the recordings. Thats were a post proccesing step comes in: marker reconstruction.

This can be done with neural networks. Some of the metrics used for evaluating reconstruction: MSE, Bone Distance Perservation (bone length), velocity distance. The goal is to compare the reconstructed movement and the groundtruth.

System considered: ground truth (manually cleaned), additive noise, vanilla CNN, Hips Outwards.
The preddiction process goes by predicting: groundtrugh, predict hips, overwrite, predict torso, overwrite, predict head, overwrite, predict limbs, overwrite.

### Conclusions
This method shows a very good results in subjective metrics and the hierarchical perdictions helps a lot and it is better than preddicting all at once.

Objective evaluation results doent correlate with subjective results. If each marker is moved one centimiter the objective notes a big difference while the subjective ones seeems the sequence good.

The objective and subjective metrics had a preaty good correlation for some of the metrics, but RMSE is the most used one and it is proved by the results that is not the best for this task.

There is no perfect groundthruth, RMSE is not a good metric as it is not correlated with subjective metrics, spatial and temporal cohesion should be taken into account.


eneko.atxa@ehu.eus