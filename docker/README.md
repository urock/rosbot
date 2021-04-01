# Docker 
## Install Guide
- Install dependecies
```
./dependencies.sh
```

- Build image
```
./build.sh
```

- Build & Run container
```
./run.sh
```

## Usage Guide
- Start container
```
docker start control
```

- Attach to container
```
docker attach control
```

- Open bash session in running container
```
docker exec -it control bash 
```

## Zsh Theme Configuration
In Container
```
p10k configure
```
