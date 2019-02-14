from site-nexus.labs.isgs.lmco.com:8080/ubuntu

COPY maps/dao /maps/dao
RUN ulimit -c 0
COPY test/environments/instances /hog2/instances
COPY test/environments/movement /hog2/movement
COPY bin/release /hog2/bin
VOLUME /output
COPY test.sh /test.sh

CMD /test.sh
