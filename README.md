# Workbench_Organizer
Workbench Organizer è un progetto sviluppato in ROS che permette di utilizzare un braccio robot antropomorfo (UR5) e una telecamera per riconoscere, tenere traccia, organizzare gli oggetti presenti nel workbench. Il sistema permette di:
- Riconoscere vari tipi di oggetti e la loro posizione sul workbench
- Muovere gli oggetti all'interno dello spazio 
- Ordinare gli oggetti secondo varie direttive
La strumentazione utilizzata comprende un robot manipolatore installato su un piano di lavoro rettangolare (un banco) e una telecamera posizionata poco distante. 
Il progetto viene sviluppato interamente in C++ e utilizza [Locosim](https://github.com/mfocchi/locosim) per la gestione del robot e l'aquisizione delle immagini. 
## Struttura del progetto 
Il progetto consiste nella realizzazione di un sistema basato su diversi nodi di ROS, che comunicano fra loro tramite messaggi. 
<p align="center">
    <img src="/documentation/Main_system.png" alt="Main System structure">
</p>
Questo progetto comprende l'implementazione dei seguenti layers:
- Manipulator control program
- Workbench manager
- Object detection system 
- Motion planner 

### Manipulator control program 
Il manipulator control program è un programma che ha il compito di creare un'interfaccia intermedia per [Locosim](https://github.com/mfocchi/locosim) in modo da semplificare i movimenti assegnati al robot. Dato che [Locosim](https://github.com/mfocchi/locosim)

