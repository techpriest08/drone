import matplotlib
# QtAgg 백엔드를 명시적으로 지정합니다. (PyQt 설치 필수)
matplotlib.use('QtAgg') 

import control
import matplotlib.pyplot as plt
import numpy as np

# 1. 시스템 및 PI 제어기 설정
# 가정된 플랜트: G(s) = 1/s^2
# PI 제어기 비율: a = Ki/Kp = 5 (-> 제어기 영점은 s = -5)
A_RATIO = 5.0

# 2. 루트 로커스용 전달함수 (L'(s)) 정의
# L'(s) = (s + A_RATIO) / s^3
numerator = [1, A_RATIO]
denominator = [1, 0, 0, 0]

sys = control.TransferFunction(numerator, denominator)

# 3. 루트 로커스 계산 및 플롯 생성
# klist, rlist를 받지 않고, plot=True만 사용합니다.
control.root_locus(sys, plot=True, xlim=[-10, 2], ylim=[-10, 10]) 

# 4. 커서 좌표 및 상수값 표시 기능 구현
fig = plt.gcf()
ax = plt.gca()

# 텍스트 객체 초기화 (좌표 및 Kp, Ki 표시용)
coordinate_text = ax.text(0, 0, '', transform=ax.transData, fontsize=10, 
                          verticalalignment='bottom', horizontalalignment='left', 
                          bbox={'facecolor': 'yellow', 'alpha': 0.8, 'pad': 4})

def calculate_gain(s_complex):
    """
    크기 조건 |K * L(s)| = 1 을 이용하여 Kp를 계산합니다.
    Kp = | 1 / L'(s) | = | s^3 / (s + a) |
    """
    s = s_complex
    
    # 전달함수의 분모: s + a = s + 5
    denom_poly = s + A_RATIO
    # 전달함수의 분자: s^3
    num_poly = s**3
    
    # L'(s)의 복소수 값을 계산
    l_s = num_poly / denom_poly
    
    # Kp = |1 / L'(s)| = |l_s|
    # 복소수 l_s의 크기를 구합니다.
    Kp = np.abs(l_s)
    
    # Ki 계산
    Ki = A_RATIO * Kp
    
    return Kp, Ki

def on_move(event):
    if event.inaxes:
        # 커서 위치의 데이터 좌표 (복소수 s)
        x = event.xdata
        y = event.ydata
        s_complex = complex(x, y)
        
        # 궤적 근처의 점을 선택했을 경우에만 Kp, Ki를 계산합니다.
        # (궤적 밖의 점은 의미가 없으므로)
        
        # 1. Kp, Ki 계산
        Kp_val, Ki_val = calculate_gain(s_complex)

        # 2. 표시할 텍스트 포맷팅
        if abs(y) < 1e-4:
             # 실수축상의 점
            s_value = f"s = {x:.3f}"
        else:
            # 복소수 영역의 점
            sign = '+' if y >= 0 else ''
            s_value = f"s = {x:.3f} {sign} j{abs(y):.3f}"
        
        # 최종 표시할 문자열
        display_text = (
            f"{s_value}\n"
            f"Kp: {Kp_val:.3f}\n"
            f"Ki: {Ki_val:.3f}"
        )
            
        coordinate_text.set_position((x, y))
        coordinate_text.set_text(display_text)
        
        fig.canvas.draw_idle()

# 6. Figure에 마우스 이동 이벤트 핸들러 연결
fig.canvas.mpl_connect('motion_notify_event', on_move)

# 7. 그래프 창 표시
plt.show()