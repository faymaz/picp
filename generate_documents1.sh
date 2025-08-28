#!/bin/bash -
#===============================================================================
#
#          FILE: generate_documents1.sh
#
#         USAGE: ./generate_documents1.sh
#
#   DESCRIPTION:
#
#       OPTIONS: ---
#  REQUIREMENTS: ---
#          BUGS: ---
#         NOTES: ---
#        AUTHOR: YOUR NAME (),
#  ORGANIZATION:
#       CREATED: 28.08.2025 00:41:50
#      REVISION:  ---
#===============================================================================

set -o nounset                                  # Treat unset variables as an error

# GitHub reposu ve branch bilgisi
REPO="https://github.com/faymaz/picp.git"
BRANCH="dev"
TEMP_DIR="/tmp/picp_temp_$(date +%s)"

# Çıkış dosyası
OUTPUT_FILE="/tmp/picp_documents.txt"

# Temizleme fonksiyonu
cleanup() {
    rm -rf "$TEMP_DIR"
    rm -f "$OUTPUT_FILE"
}

# Hata yakalama
trap cleanup EXIT

# Repoyu klonla
echo "GitHub reposunu klonluyor..."
git clone --branch "$BRANCH" --depth 1 "$REPO" "$TEMP_DIR" 2>/dev/null
if [ $? -ne 0 ]; then
    echo "Hata: Repoyu klonlayamadı. İnternet bağlantısını veya repo adresini kontrol et."
    exit 1
fi

# Önemli dosyaları liste (dizi yerine basit liste)
IMPORTANT_FILES="main.c k150.c k150.h k150_config.c Makefile picdevrc K150_FUSE_PROGRAMMING_GUIDE.md K150_FUSE_READ_GUIDE.md debug_picp.log"

# Çıktı dosyasını oluştur
echo "Dosyaları <DOCUMENT> formatına dönüştürüyor..."
> "$OUTPUT_FILE"  # Boş dosya oluştur

for file in $IMPORTANT_FILES; do
    if [ -f "$TEMP_DIR/$file" ]; then
        echo "<DOCUMENT filename=\"$file\">" >> "$OUTPUT_FILE"
        # İlk 1000 satırı al (büyük dosyalar için kesme)
        head -n 1000 "$TEMP_DIR/$file" >> "$OUTPUT_FILE" 2>/dev/null
        echo "</DOCUMENT>" >> "$OUTPUT_FILE"
        echo "Dosya işlendi: $file"
    else
        echo "Uyarı: $file bulunamadı, atlanıyor."
    fi
done

# Çıktıyı ekrana yazdır (kopyala ve buraya yapıştır)
cat "$OUTPUT_FILE"

echo "Tamamlandı! Çıktıyı kopyalayıp buraya yapıştırabilirsiniz."
