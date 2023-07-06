# Start with the official ROS 2 + MoveIt 2 image
FROM moveit/moveit2:humble-release

# Set the working directory
WORKDIR /app

# Install any additional system packages needed
RUN DEBIAN_FRONTEND=noninteractive apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y \
    build-essential \
    git \
    cmake \
    vim \
    wget \
    curl \
    gnupg \
    ca-certificates \
    software-properties-common \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# RUN apt-get update && apt-get install -y --no-install-recommends \
#     qtbase5-dev \
#     qtwayland5

# Renderer
RUN apt-get update && apt-get install -y mesa-utils && apt-get clean && rm -rf /var/lib/apt/lists/*
RUN apt-get update && apt-get install -y libgl1-mesa-glx libgl1-mesa-dri
# ENV LIBGL_ALWAYS_SOFTWARE=1
# Below line worked. Down to one Error: glXCreateContext failed
ENV LIBGL_ALWAYS_INDIRECT=1

# RUN apt-get update && apt-get install -y xvfb

# RUN apt-get update && apt-get install -y xauth strace xterm mesa-utils
# ENV QT_X11_NO_MITSHM 1

# # # Open3D
# # RUN pip3 install --user open3d

# # # Install PCL and Eigen
# # RUN apt-get install -y libpcl-dev libeigen3-dev

# # # Install pip packages from PyPi
# # RUN pip3 install \
# #     openai \
# #     transformers

# # RUN pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118

# # # Install all Hugging Face libraries (note, some might already be included in 'transformers' package)
# # RUN pip3 install \
# #     datasets \
# #     tokenizers \
# #     sacremoses

# # # Install Python robotics packages
# # RUN pip3 install pykdl_utils pybullet

# # # Add requirements.txt file to container
# # COPY docker/requirements.txt /

# # # Install python packages from requirements.txt
# # RUN pip3 install -r /requirements.txt

# # # Install Chrome (headless)
# # RUN wget https://dl.google.com/linux/direct/google-chrome-stable_current_amd64.deb
# # RUN dpkg -i google-chrome-stable_current_amd64.deb; apt-get -fy install

# Clean up
RUN apt-get clean && rm -rf /var/lib/apt/lists/*

# Copy the rest of the application code into the container
COPY . .

RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc
RUN echo "alias python=python3" >> /root/.bashrc

RUN chmod 755 /app/docker/entrypoint.sh

ENTRYPOINT ["bash",  "/app/docker/entrypoint.sh" ]
