SCRIPT=$(realpath "$0")
SCRIPTPATH=$(dirname "$SCRIPT")

docker build -t dronecode/roscon-25-workshop -f ${SCRIPTPATH}/Dockerfile  ${SCRIPTPATH}/..