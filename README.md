# CARLA V2V Research Platform

### Before setting up the Framework Install Carla

1. Download a pre compiled version of Carla 0.9.16 either for windows or for a Ubuntu workstation. [https://github.com/carla-simulator/carla/releases]  

2. Unzip/Extract the Carla Folder
3. Start a Terminal to start the carla server inside carla dir run `.\CarlaUE4.exe -carla-rpc-port=2000`
4. when using a windows computer as carla server maybe port 2000 is not reachable from outside by default. To make port 2000 reachable when on windows run the command below in admin powershell.
      - ``` New-NetFirewallRule -DisplayName "Carla Server" -Direction Inbound -Protocol TCP -LocalPort 2000 -Action Allow -Profile Private,Domain ```




### Set up with Docker
Just use the following commands as intended

`docker-compose build`       # Build Container
`docker-compose up -d`       # Start Container

You are able now to access the frontend Dashboard unter [[localhost:8000](http://localhost:8000/)]

`docker-compose logs -f`     # View logs
`docker-compose down`        # Stop and remove



### Setting up the workspace
I recommend to use the Dockerfile to set everything up!
    
1. setup a virtual environment of your choice using the requirements.txt file (`pip install -r requirements.txt`; for development also `pip install pytest httpx ruff pyright`, or `pip install -e ".[dev]"`)

2. start venv and run the `start_server.py` script. You can reach the frontend dashboard on [[localhost:8000](http://localhost:8000/)] 

3. after accessing the frontend dashboard you can set the Server adress of the computer that will run the actual carla server for you

4. You can now start the scenario from the frontend Dashboard.

---

## ER Database Model

![ER Database Model](docs/ER_DATABASE_MODEL.png)
