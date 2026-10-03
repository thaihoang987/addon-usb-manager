ARG BUILD_FROM=ghcr.io/home-assistant/base:3.23
FROM $BUILD_FROM

RUN apk add --no-cache python3 py3-pip tzdata
WORKDIR /app
COPY requirements.txt ./requirements.txt
RUN pip install --no-cache-dir --break-system-packages -r requirements.txt
COPY ha-addon-usb-manager.py config.yaml ./
EXPOSE 6001-6010
CMD ["python3", "ha-addon-usb-manager.py"]
