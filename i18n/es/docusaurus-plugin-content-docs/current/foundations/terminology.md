---
title: "Terminología y Modelos de Sistema"
module: "foundations"
word_count: 380
learning_objectives:
  - "Definir la terminología clave utilizada en Robótica AI"
  - "Comprender los modelos de sistema fundamentales"
  - "Distinguir entre conceptos relacionados"
prerequisites:
  - "Conocimiento básico de conceptos de IA"
  - "Familiaridad con fundamentos de ciencias de la computación"
references:
  - "@siciliano2016springer"
validation_status: draft
---

# Terminología y Modelos de Sistema en Robótica AI

## Terminología Clave

### IA Física
La IA Física se refiere a sistemas de inteligencia artificial que trabajan en el mundo real, a diferencia de la IA solo digital. La IA Física debe lidiar con restricciones del mundo real como física, ruido de sensores y seguridad.

### Inteligencia Encarnada
La inteligencia encarnada significa que la inteligencia proviene de cómo un sistema interactúa con su entorno físico. Esto muestra que el cuerpo y sus interacciones son partes clave del comportamiento inteligente.

### Robot Operating System (ROS)
ROS es un framework flexible para escribir software de robot. Proporciona servicios como abstracción de hardware, controladores de dispositivos, bibliotecas y paso de mensajes. ROS 2 es la versión más nueva de este framework.

### Gemelo Digital
Un gemelo digital es un modelo virtual de un sistema físico. Puede utilizarse para simulación, pruebas y validación antes de usar hardware real. En robótica, los gemelos digitales permiten pruebas seguras.

### Transferencia Sim-a-Real
El proceso de mover comportamientos o modelos entrenados en simulación a robots reales. Esto a menudo requiere técnicas para tener en cuenta las diferencias entre simulación y realidad.

### Sistemas Visión-Lenguaje-Acción (VLA)
Sistemas integrados que toman entrada visual y comandos de lenguaje para generar acciones físicas. Estos sistemas combinan percepción, comprensión del lenguaje y control robótico.

## Modelos de Sistema

### Bucle Percepción-Acción
Un modelo en robótica que describe el ciclo de sensar el entorno, procesar información, tomar decisiones y realizar acciones. Este bucle es la base de los comportamientos robóticos.

### Modelo de Estimación de Estado
Un modelo que muestra la comprensión del robot de su estado interno (posición, velocidad) y el entorno basado en datos de sensores. Los enfoques comunes incluyen filtros de Kalman.

### Arquitectura de Control
La estructura que determina cómo se toman las decisiones de control en un sistema robótico. Esto incluye arquitecturas basadas en planificación y arquitecturas basadas en comportamiento.

### Computación Morfológica
Un modelo que muestra que las propiedades físicas del cuerpo de un robot pueden simplificar el control. Por ejemplo, articulaciones flexibles pueden adaptarse al terreno sin algoritmos complejos.

## Conceptos Erróneos Comunes

- **IA vs. Robótica**: La IA proporciona la inteligencia, mientras que la robótica proporciona la forma física. La Robótica AI combina ambos.
- **Autonomía vs. Automatización**: Los sistemas autónomos pueden adaptarse a nuevas situaciones, mientras que los sistemas automatizados siguen secuencias fijas.
- **Simulación vs. Realidad**: La simulación perfecta es imposible; la transferencia sim-a-real debe tener en cuenta las diferencias.

## Resumen

Comprender esta terminología y modelos de sistema proporciona la base para temas más avanzados en robótica AI. Estos conceptos se utilizarán a lo largo del libro a medida que exploremos implementaciones y aplicaciones específicas.