
import os
import subprocess
from gtts import gTTS
import time
import atexit
import config_2 as cfg
from config import AGVConfig
import random
from config_2 import AGVConfig_2

path_folder_mp3 = cfg.PATH_PHAN_MEM + "/mp3"


name_music = ""
name_music_old = ""
start_sound = 0
new_thread = 1
connect_sound = True
time_reset_none = 0
time_reset = 0
file_music_old = ""


# pkill mpg123

# hoặc nếu không có pkill:

# killall mpg123

# Hoặc thủ công hơn:

# ps aux | grep mpg123
# kill <PID>

time_continue = 0
save_time_continue = 0
temp_file = ""
temp_file_old = ""



data = {"loi_motor_nang_ha": 0,
        "dung_hoat_dong": 0,
        "vung_1": 0,
        "bat_dau_di_chuyen": 0,
        "vung_2": 0,
        "vung_3": 0,
        "dang_ha_hang": 0,
        "dang_nang_hang": 0,
        "lui_phai": 0,
        "lui_trai": 0,
        "re_phai": 0,
        "re_trai": 0,
        "sap_den_dich": 0,
        "het_pin": 0,
        "doi_dau_di_chuyen": 0,
        "dang_den_vi_tri_cho": 0,
        "da_den_dich": 0,
        "icp_sai_vi_tri": 0}



thu_tu_uu_tien = [["loi_motor_nang_ha", "icp_sai_vi_tri"],
                  ["dung_hoat_dong"],  
                  ["vung_1"], 
                  ["bat_dau_di_chuyen"], 
                  ["vung_2"], 
                  ["vung_3"], 
                  ["dang_ha_hang", "dang_nang_hang", "lui_phai", "lui_trai", "re_phai", "re_trai"], 
                  ["doi_dau_di_chuyen", "dang_den_vi_tri_cho", "sap_den_dich", "het_pin", "da_den_dich"]]


thoi_gian_phat_nhac = {"2": ["lui_phai", "lui_trai"],
                       "3": ["bat_dau_di_chuyen", "doi_dau_di_chuyen"],
                       "4": ["da_den_dich", "re_phai", "re_trai", "dang_den_vi_tri_cho", "vung_2", "vung_3", "sap_den_dich"],
                       "5": ["co_vat_can", "dang_ha_hang", "het_pin", "dang_nang_hang", "dung_hoat_dong", "loi_motor_nang_ha", "icp_sai_vi_tri"],
                       "10": ["vung_1"],
                       "300": ["e_cua_ngay_hom_qua"]}
# index = 0
index = random.randint(0, len(thoi_gian_phat_nhac["300"]) - 1)
name_none = thoi_gian_phat_nhac["300"][index]

time_next = time.time()
check_time = time.time()
def check_connect():
    global check_time
    check_time = time.time()
def xu_ly_du_lieu():
    global name_music, name_music_old, connect_sound
    if time.time() - check_time > 2 or AGVConfig.tat_phan_mem == True:
        if connect_sound == True:
            stop()
        connect_sound = False
    else:
        connect_sound = True
        for index, value in data.items():
            name_none = 1
            if value == 1:
                name_none = 0
                break
        if name_none == 1:
            name_music = "none"
        else:
            # print(data)
            for i in range(0,len(thu_tu_uu_tien)):
                # kiểm tra xem có giá trị nào cùng thứ tự ưu tiên đang phát hay không
                # check_name_music = False
                if name_music in thu_tu_uu_tien[i] and data[name_music] == 1:
                    break
                name_music_new = "none"
                for j in range(0,len(thu_tu_uu_tien[i])):
                    if data[thu_tu_uu_tien[i][j]] == 1:
                        name_music_new = thu_tu_uu_tien[i][j]
                        break
                if name_music_new != "none":
                    name_music = name_music_new
                    break
    
            


            

    
def kiem_tra_thoi_gian(name):
    for index, data in thoi_gian_phat_nhac.items():
        if name in data:
            return int(index)
    return 300

def la_nhac_nen(name):
    """Kiểm tra xem một bài hát có phải là nhạc nền không."""
    if (name in thoi_gian_phat_nhac["300"]) or name == "none":
        return True
    return False

# Biến toàn cục để giữ process đang chạy
_player_proc = None

def play(filepath: str, resume_from_seconds: int = 0):
    """
    Phát file mp3 theo tên (không có đuôi .mp3).
    Nếu đang phát nhạc khác thì dừng trước rồi mới phát.
    """
    global _player_proc
    stop()  # dừng nhạc cũ trước
    print(f"[music] Playing,,,,,,,,,,,,,,")

    if not os.path.exists(filepath):
        print(f"[music] File not found: {filepath}")
        return

    try:
        command = ["mpg123", "-q"]
        if resume_from_seconds > 0:
            # Chuyển đổi giây thành số khung (frame) để tua.
            # Giả định MP3 có 1152 sample/frame và sample rate là 48000Hz (phổ biến).
            # Frames per second = 48000 / 1152 ≈ 41.67
            frames_to_skip = int(resume_from_seconds * (48000 / 1152))
            print(f"[music] Resuming from {resume_from_seconds}s, skipping {frames_to_skip} frames.")
            command.extend(["-k", str(frames_to_skip)])
        command.append(filepath)
        _player_proc = subprocess.Popen(command)
        print(f"[music] Playing: {filepath}")
    except FileNotFoundError:
        print("[music] Error: mpg123 not found. Cài bằng: sudo apt install mpg123")
    except Exception as e:
        print(f"[music] Error starting player: {e}")

def stop():
    """
    Dừng nhạc đang chạy (nếu có).
    """
    global _player_proc
    if _player_proc:
        try:
            _player_proc.terminate()
            _player_proc.wait(timeout=1)
            print("[music] Stopped.")
        except Exception:
            try:
                _player_proc.kill()
            except:
                pass
        finally:
            _player_proc = None

# Đăng ký hàm stop() để tự động chạy khi chương trình kết thúc
# Điều này đảm bảo tiến trình mpg123 luôn được dọn dẹp sạch sẽ.
atexit.register(stop)

def creat_music(text, name, lang='vi'):
    temp_file = path_folder_mp3 + "/" + name + ".mp3"
    if os.path.exists(temp_file) == False:
        tts = gTTS(text=text, lang=lang)
        tts.save(temp_file)
# Hàm để phát âm thanh 240
def void_loop_sound_speak():
    global name_music, connect_sound, name_music_old, time_reset, time_continue, time_reset_none, save_time_continue, time_next, name_none, index, file_music_old
    global temp_file, temp_file_old
    if connect_sound and AGVConfig.thiet_lap_ket_noi["esp32"] == "on":
        play_condition = False
        is_background_music = la_nhac_nen(name_music) # kiem tra co phai nhac 240s không

        if is_background_music:
            # Nhạc nền chỉ phát khi đổi bài, không phát lại theo thời gian
            play_condition = (temp_file != temp_file_old)
            time_continue = save_time_continue + int(time.time() - time_reset_none)
            if time_continue >= 300: # Nếu lớn hơn thời lượng bài hát thì reset
                time_continue = 0
                save_time_continue = 0
                play_condition = True
                time_reset_none = 0
            if time_reset_none == 0:
                time_reset_none = time.time()
                time_continue = 0
        else:
            save_time_continue = time_continue
            # Âm thanh cảnh báo phát lại khi đổi hoặc hết thời gian chờ 
            play_condition = (name_music_old != name_music or time.time() - time_reset > kiem_tra_thoi_gian(name_music))
            time_reset_none = time.time()

        # print(time_continue)
                

        if play_condition and name_music != "":
            resume_time = 0
            # print(is_background_music, time_continue)
            # Nếu bài hát cũ là nhạc nền và bài mới không phải là nhạc nền, lưu lại thời gian đã phát
            if is_background_music:
                resume_time = time_continue

            time_reset = time.time()
            name_music_old = name_music

            # print("name_music", name_music)
            
            temp_file = path_folder_mp3 + "/" + name_music + ".mp3"
            if os.path.exists(temp_file) == False:
                if time.time() - time_next > 300:
                    time_next = time.time()
                #     # index = index + 1
                    index = random.randint(0, len(thoi_gian_phat_nhac["300"]) - 1)
                    name_none = thoi_gian_phat_nhac["300"][index % len(thoi_gian_phat_nhac["300"])]
                    time_continue = 0
                    save_time_continue = 0
                    play_condition = True
                    time_reset_none = 0
                    resume_time = 0

                temp_file = path_folder_mp3 + "/" + name_none + ".mp3"
            if os.path.exists(temp_file) == True:
                if name_music == "none":
                    if temp_file != file_music_old:
                        play(temp_file, resume_from_seconds=resume_time)
                        file_music_old = temp_file
                else:
                    play(temp_file, resume_from_seconds=resume_time)
                    file_music_old = temp_file
            else:
                # stop()
                print("không có tên bài hát: " +temp_file )
    else:
        stop()
                

# if __name__ == "__main__":
#     main()

# creat_music("車は右折しています", "re_phai_japan","ja")
# creat_music("車は左折しています", "re_trai_japan","ja")
# creat_music("車は右に後退している", "lui_phai_japan","ja")
# creat_music("車は左に後退している", "lui_trai_japan","ja")
# creat_music("障害がある", "vat_can_japan","ja")

# creat_music("đang rẽ phải", "re_phai","vi")
# creat_music("đang rẽ trái", "re_trai","vi")
# creat_music("đang lùi phải","lui_phai","vi")
# creat_music("đang lùi trái", "lui_trai","vi")
# creat_music("có vật cản", "co_vat_can","vi")
