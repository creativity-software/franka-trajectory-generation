 #!/bin/bash
 mkdir -p .ros1 .ros1/build .ros1/devel
 docker-compose build
 chown -R 1000:1000 .ros1
 
 
