# Kakip Linuxカーネル


## ビルド環境

[Renesas社の手順](https://renesas-rz.github.io/rzv_ai_sdk/5.00/getting_started.html)を参考にRZ/V2H用AI SDKのコンテナイメージを作成してください。

## ビルド手順

1. リポジトリのクローン

    ```
    git clone https://github.com/Kakip-ai/kakip_linux
    cd kakip_linux
    ```

2. カーネルコンフィグの設定

    ```
    cp ./arch/arm64/configs/kakip.config .config
    ```

3. ビルド環境(コンテナ)の起動

    環境によってはsudoを付けて実行する必要があります。

    ```
    docker run --rm -it -v $PWD:/kakip_linux -w /kakip_linux rzv2h_ai_sdk_image
    ```

4. 環境変数の設定と依存パッケージのインストール

    ```
    source /opt/poky/3.1.31/environment-setup-aarch64-poky-linux
    apt update && apt install -y flex bison bc
    ```

5. ビルド

    ```
    make -j4 Image
    make -j4 renesas/kakip-es1.dtb
    ```

    ビルド成果物は以下の2点です。

    - ./arch/arm64/boot/Image
    - ./arch/arm64/boot/dts/renesas/kakip-es1.dtb

    ビルド後はexitでコンテナから抜けて下さい。

    ```
    exit
    ```

## カーネルの更新

Kakipのイメージが書き込まれているSDカードのファイルを更新します。

1. SDカードをPCにマウントする

    /mntに手動でマウントする場合の手順です。  
    自動マウントされる環境の場合は、以降マウント先のパスを読み替えてください。

    ```
    # sd<X>は環境によります。
    sudo mount /dev/sd<X>2 /mnt
    ```

2. ビルドしたカーネルイメージとdtbファイルを更新する

    ```
    sudo cp ./arch/arm64/boot/Image /mnt/boot/Image-5.10.145-cip17-yocto-standard
    sudo cp ./arch/arm64/boot/dts/renesas/kakip-es1.dtb /mnt/boot/kakip-es1.dtb 
    ```

3. SDカードをPCからアンマウントする

    ```
    sudo umount /mnt
    ```
