### Deployment Instructions
1. cd /Release && scp libslam.so admin@172.16.0.1:~/lib/libslam.so
2. ssh admin@172.16.0.1
3. cd ~/lib && ldconfig
4. exit

note: Deploying to the rio crashes once after updating the library sometimes. This is probably fine.