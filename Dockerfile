FROM adityang5/fsds:latest
# FROM fsds_build:v1

COPY . /app

COPY settings.json /fsds/Formula-Student-Driverless-Simulator/settings.json

# CMD ["/bin/sh", "/app/run.sh"]

