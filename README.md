# Islandshast_simulator
Kandidatarbete vid Institutionen för fysik på Chalmers. Kandidatarbetet har gjorts i samarbete med Axevalla ridcentrum, Hólar University och Wången. Syftet med arbetet är att utveckla och validera en fysisk prototyp av en ridsimulator som kan reproducera sadelns rörelsemönster under gångarten tölt, baserat på mätdata insamlad från islandshäst.

# Innehåll
Data från tölt hos 2 islandshästar: se raw data-mappen.

Kod för simulering av en Stewart-plattform.

Kod för att kunna köra vår fysikaliska Stewart-plattform.

MATLAB-kod för att kunna titta på och hantera den råa datan. 

# För att köra koden:
1. Run the command uv sync in the terminal for the project
2. Om koden ska köras på simulatorn behöver även platform IO extension till VS code vara installerat (Adruino Due mappen) och en Usb kabel behöver vara inkoplad i datorn som i andra enden är inkoplad til Adruino Due programeringsport (microUsb)  
3. main.py eller main_simulering.py kan nu köras. Inställningar finns att justera i config.py (vilken data som simuleras o.s.v.). I main.py kan man även justera:

   SEND_TO_ARDUINO = False #om simuleringen ska köras på riggen
   
   simulering_köras = True #om den visar en virituel simuleringen av platfomens rörelser och plottar datan
   
   Sinus_rörelse = False #om en sinus rörelse ska simuleras iställe för datan
