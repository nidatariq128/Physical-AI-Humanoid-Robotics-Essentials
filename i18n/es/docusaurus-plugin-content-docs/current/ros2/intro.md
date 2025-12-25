---
title: "Introducción a ROS 2"
module: "ros2"
word_count: 450
learning_objectives:
  - "Comprender la arquitectura y conceptos de ROS 2"
  - "Explicar las diferencias entre ROS 1 y ROS 2"
  - "Identificar los componentes clave del ecosistema de ROS 2"
prerequisites:
  - "Comprender conceptos básicos de Robótica AI"
  - "Familiaridad con fundamentos de ciencias de la computación"
references:
  - "@quigley2009ros"
  - "@macenski2022ros2"
validation_status: draft
---

# Introducción a ROS 2

## Visión general

ROS 2 (Robot Operating System 2) es el framework de próxima generación para desarrollar aplicaciones de robot. Aborda las limitaciones de ROS 1 y proporciona soporte mejorado para despliegue en el mundo real, seguridad y sistemas multi-robot. A diferencia de ROS 1, que se construyó sobre un modelo de red peer-to-peer, ROS 2 usa middleware moderno basado en Data Distribution Service (DDS) para una comunicación robusta.

## Arquitectura y Conceptos

### Nodos
En ROS 2, un nodo es un proceso que realiza cálculos. Los nodos son los bloques de construcción fundamentales de un sistema ROS 2. Cada nodo puede realizar funciones específicas y comunicarse con otros nodos a través de tópicos, servicios y acciones. Los nodos típicamente se escriben en C++ o Python y pueden iniciarse individualmente o como parte de un sistema más grande.

### Tópicos y Mensajes
Los tópicos permiten la comunicación asincrónica entre nodos usando un patrón de publicación-suscripción. Un nodo publica datos a un tópico, mientras que otros nodos se suscriben a ese tópico para recibir los datos. Los mensajes son las estructuras de datos que se pasan entre nodos. Cada mensaje tiene un tipo específico y contiene campos con diferentes tipos de datos.

### Servicios
Los servicios proporcionan comunicación sincrónica de solicitud-respuesta entre nodos. Cuando un nodo cliente envía una solicitud a un servicio, espera una respuesta del nodo servidor. Esto es útil para operaciones que requieren una respuesta específica o confirmación de finalización.

### Acciones
Las acciones son una forma más avanzada de comunicación que soportan tareas de larga duración con retroalimentación. Permiten a los clientes enviar objetivos a servidores de acción, recibir retroalimentación durante la ejecución y obtener un resultado cuando se completa el objetivo. Las acciones son ideales para navegación, manipulación y otras tareas robóticas complejas.

## ROS 2 vs ROS 1

ROS 2 se desarrolló para abordar varias limitaciones de ROS 1:

- **Middleware**: ROS 2 usa DDS (Data Distribution Service) para comunicación, proporcionando mejor confiabilidad y rendimiento en tiempo real
- **Soporte multi-robot**: ROS 2 tiene soporte nativo para sistemas multi-robot sin conflictos de espacio de nombres
- **Seguridad**: ROS 2 incluye características de seguridad integradas para comunicación autenticada y encriptada
- **Soporte en tiempo real**: ROS 2 proporciona mejor soporte para aplicaciones en tiempo real
- **Compatibilidad multiplataforma**: ROS 2 corre en Windows, macOS y Linux, con mejor soporte para sistemas embebidos

## Componentes Clave

### Implementación de DDS
ROS 2 usa DDS como su capa de comunicación subyacente. Están disponibles diferentes implementaciones de DDS, incluyendo Fast DDS, Cyclone DDS y RTI Connext DDS. La elección de la implementación de DDS puede afectar el rendimiento, capacidades en tiempo real y requisitos de despliegue.

### Gestión de Ciclo de Vida
ROS 2 introduce nodos de ciclo de vida que proporcionan mejor gestión de estado para sistemas complejos. Esto permite inicialización, configuración y procedimientos de apagado más robustos.

### Gestión de Paquetes
ROS 2 usa colcon para construir paquetes, que es más flexible que el sistema de construcción catkin usado en ROS 1. Esto permite mejor integración con herramientas de construcción estándar y compatibilidad multiplataforma.

## Comenzando

Para trabajar con ROS 2, necesitará instalar una distribución (como Humble Hawksbill LTS) y configurar su entorno de desarrollo. Los conceptos centrales de ROS 2 permanecen consistentes a través de diferentes distribuciones, aunque paquetes y herramientas específicas pueden variar.

## Resumen

ROS 2 proporciona el "sistema nervioso" para aplicaciones robóticas, permitiendo la comunicación entre diferentes componentes de un robot. Comprender los conceptos de ROS 2 es esencial para construir sistemas robóticos complejos que puedan integrar percepción, planificación, control e interacción. Las siguientes secciones explorarán la implementación práctica de estos conceptos.