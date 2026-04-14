

from libs_file import remove 
import os, time
import config as cfg
from gtts import gTTS

path_folder_mp3 = cfg.PATH_PHAN_MEM + "/mp3"


def creat_music(text, name, lang='vi'):
    temp_file = path_folder_mp3 + "/" + name + ".mp3"
    remove.remove_file(temp_file)
    time.sleep(0.5)
    tts = gTTS(text=text, lang=lang)
    tts.save(temp_file)


if __name__ == "__main__":
    #     main()

    # creat_music("車は右折しています", "re_phai_japan","ja")
    # creat_music("車は左折しています", "re_trai_japan","ja")
    # creat_music("車は右に後退している", "lui_phai_japan","ja")
    # creat_music("車は左に後退している", "lui_trai_japan","ja")
    # creat_music("障害がある", "vat_can_japan","ja")

    # creat_music("em cua phải anh chị chú ý", "re_phai","vi")
    # creat_music("em cua trái anh chị chú ý", "re_trai","vi")
    # creat_music("em đang lùi phải","lui_phai","vi")
    # creat_music("em đang lùi trái", "lui_trai","vi")
    # creat_music("có vật cản", "vung_1","vi")

    # creat_music("anh chị giải tán em sắp phi qua", "co_vat_can","vi")
    # creat_music("Giúp em loại bỏ vật cản, em cám ơn", "vung_1_1","vi")
    # creat_music("Anh chị chú ý em sắp phi qua", "vung_2","vi")
    # creat_music("Anh chị chú ý em sắp phi đến", "vung_3","vi")
    # creat_music("sắp tới rồi sắp tới rồi", "sap_den_dich","vi")

    # creat_music("đã đến vị trí yêu cầu", "da_den_dich","vi")

    creat_music("nhận diện sai vị trí bản đồ", "icp_sai_vi_tri","vi")

    # creat_music("em đang nâng hàng, anh chị chú ý", "dang_nang_hang","vi")
    # creat_music("em đang hạ hàng, anh chị chú ý", "dang_ha_hang","vi")

    # creat_music("em sắp hết pin, anh chị sạc pin giúp em", "het_pin","vi")

    # creat_music("Lỗi động cơ nâng hạ", "loi_motor_nang_ha","vi")

    # creat_music("Em chuẩn bị di chuyển anh chị chú ý", "bat_dau_di_chuyen","vi")

    # creat_music("Em bị tác động vật lý, nhấn reset và start để chạy tiếp", "dung_hoat_dong","vi")
