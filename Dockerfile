FROM python:3.11-slim

# X11 client libs for matplotlib TkAgg backend
RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-tk \
        libx11-6 \
        libxext6 \
        libxrender1 \
        libxtst6 \
        libxi6 \
    && rm -rf /var/lib/apt/lists/*

RUN python -m venv /opt/venv
ENV PATH="/opt/venv/bin:$PATH"

COPY requirements.txt /tmp/requirements.txt
RUN pip install --no-cache-dir -r /tmp/requirements.txt

WORKDIR /workspace

# TkAgg renders via the host X server passed in at runtime via $DISPLAY
ENV MPLBACKEND=TkAgg

CMD ["bash"]
