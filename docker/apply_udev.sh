# This script applies the udev rules. 
# It should be run on the host machine, not inside the Docker container.
sudo cp ~/ai_worker/docker/99-ai-worker.rules /etc/udev/rules.d/99-ai-worker.rules
sudo udevadm control --reload-rules
sudo udevadm trigger