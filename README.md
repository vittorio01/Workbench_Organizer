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

In questo file sono presenti solamente informazioni basilari. nel progetto sono previste anche documentazioni secondarie in cui vengono specificati struttura e funzionamento nello specifico dei layers.

### Manipulator control program 
Il manipulator control program (MCP) è un programma che ha il compito di creare un'interfaccia intermedia dedicata al controllo dell'hardware, in modo da semplificare i movimenti assegnati al robot. La struttura di Locosim permette infatti di modificare direttamente l'angolazione delle giunture del robot a partire da informazioni specifiche (posizione finale, velocità, ecc ...). 
Questo layer cerca di semplificare tutte le operazioni di movimento, permettendo ai layer che utiizzano il robot di controllarlo tramite informazioni più ristrette, come la posizione finale dell'end effector e la sua inclinazione.

L'MCP agisce attivamente su tutte le dinamiche relative ai movimenti basilari del robot, provvedendo alla realizzazione di un sistema che implementa tutti i principi della cinematica differenziale e gestisce anche eventuali problemi nei movimento. 

<p align="center">
    <img src="/documentation/end_effector_position.png" alt="Main System structure">
</p>

Da un punto di vista ideale, il programma deve implementare funzioni del tipo:
- `RobotState setEndEffector_position(x, y, z, a)`, per muovere l'end effector nella posizone `(x, y, z)` inclinato di un angolo `a` rispetto alla normale.
- `(x,y,z,a) getEndEffector_position()`, per ricevere la posizione corrente dell'end effector (sempre nel formato `(x, y, z, a)`)
- `GripState get_gripState()`, per capire lo stato dell'end effector
- `GripState set_gripState()`, per modificare lo stato dell'end effector

Le funzioni verranno successivamente richiamate dal Workbench Manager tramite messaggi di ROS.

### Object detection system

L'object detection system (ODS) è un programma che ha il compito di riconoscere gli oggetti presenti sul workbench e individuarne la loro posizione. 
La posizione degli oggetti viene identificata utilizzando delle coordinate specifiche.

<p align="center">
    <img src="/documentation/object_position.png" alt="Main System structure">
</p>

La telecamera viene posizionata e calibrata in una posizione fissa rispetto al banco di lavoro e invia periodicamente immagini da analizzare. L'OSD deve quindi provvedere ad analizzare le immagini ricevute restituendo informazioni tra cui:
- Numero di oggetti nel workbench.
- tipo di oggetti.
- posizione nel piano di lavoro per ogni oggetto `(x, y, z, a)`.

A sua volta, l'ODS deve inviare periodicamente messaggi contenenti informazioni sulle informazioni ricavate, che verranno ricevute e organizzate dal Workbench manager. 

### Workbench manager 

Il Workbench Manager ha una duplice funzione:
- Tenere traccia di tutti gli oggetti presenti nel workbench 
- Cambiare posizione agli oggetti 

Da un punto di vista pratico, il WM è un programma basato sulla programmazione ad oggetti che genera una rappresentazione dello spazio e si interfaccia con ODS e MCP per la gestione di tutti i suoi elementi. Se da un lato riceve tutti i messaggi dall'ODS per aggiungere, rimuovere, verificare i movimenti degli oggetti, dall'altro utilizza la MCP per cambiare la loro posizione all'interno dello spazio di lavoro. Il WM deve quindi implementare le seguenti funzionalità:
- creare una lista di oggetti in cui vengono inserite informazioni relative al tipo, alla forma, alla posizione di essi.
- implementare la funzione `ObjectState move(Object o, x, y, z, a)`, per muovere gli oggetti all'interno del workspace. Questa funzione verrà successivamente richiamata dal Motion Planner tramite messaggi di ROS, che utilizzerà il robot per spostare gli oggetti.

L'esecuzione del WM procede in due fasi principali:
- Fase di inizializzazione, in cui il sistema provvede all'inizializzazione della lista degli oggetti presenti nel workspace e comunica al Motion Planner tutte le infomazioni che è riuscito a ricavare (sempre tramite messaggi di ROS)
- Fase di Servizio, in cui il sistema riceve i messaggi dal Motion Planner e muove effettivamente gli oggetti nel workspace tramite la funzione `move`. 

### Motion Planner 
Il Motion Planner rappresenta la parte intelligente del progetto. Date le posizioni di partenza (ricvute dal WM) e dato un target che identifica la struttura finale desiderata (può essere una costruzione o semplicemente una lista di posizioni finali), il Motion Planner ha il compito di pianificare tutti i movimenti, che dovranno essere successivamente applicati dal WM e, quindi, dal robot. dato che il MP è la parte ad alto livello del progetto che comunica direttamente con l'utente, l'idea sarebbe di creare un'interfaccia CLI in che permette di scegliere il compiti da eseguire oppure permettere all'utente di inviare direttamente un target di posizioni. 

La programmazione di questo layer si basa, a seconda della sua struttura, nella risoluzione di problemi di ottimizzazione relativi allo spostamento degli oggetti (si vuole far in modo di spostare gli oggetti nel modo più veloce possibile). 

## Sviluppo del progetto 

Il progetto, dato che viene suddiviso in più sezioni, può essere completato a partire da programmi indipendenti fra loro. Successivamente, una volta stabiliti i formati dei messaggi, verranno interfacciati per funzionare con ROS e, quindi comunicare fa loro. Da un punto di vista cronologico, lo sviluppo dei layers viene suddiviso in due fasi:
- Fase 1, in cui tutti i componenti del gruppo lavorano contemporaneamente allo sviluppo del source code di MSP, ODS e WM
- Fase 2, (ancora da stabilire) in cui una/due persone lavorano al MP e il resto dei componenti all'organizzazione della docuentazione finale 

Durante la prima fase, tutti i componenti del gruppo hanno il compito di tener traccia di tutti gli algoritmi/procedure utilizzate per lo sviluppo, commentandolo in modo esaustivo e creando un documento (il formato è arbitrario) in cui vengono inserite tutte le informazioni (che verranno poi elaborate nella seconda fase e studiate dagli altri componenti del gruppo). 

## Contenuto del progetto 
In un primo momento, il progetto GitHub viene suddiviso in quattro cartelle principali in cui ogni Layer viene sviluppato separatamente. Ogni cartella contiene obbligatoriamente una sottocartella Doc in cui viene inserita tutta la documentazione relativa (tutto il resto delle cartelle può essere utilizzato a piacimento). 
Successivamente, tutto il codice verrà trasformato in un vero e proprio progetto ROS e le documentazioni rielaborate in un unico file descrittivo. 

**Tutti i componenti del gruppo non devono modificare il contenuto delle altre cartelle, per evitare problemi di sviluppo!**
