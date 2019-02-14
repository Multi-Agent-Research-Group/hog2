#!/bin/bash -x

rm -rf bin/release
docker cp hog2:/hog2/bin/release ./bin/release
docker build -t site-nexus.labs.isgs.lmco.com:8080/mapf .
docker push site-nexus.labs.isgs.lmco.com:8080/mapf
for ((i=0; i<10; i++)); do
  ssh mst${i} docker pull site-nexus.labs.isgs.lmco.com:8080/mapf
  ssh mst${i} docker image prune -f
done
