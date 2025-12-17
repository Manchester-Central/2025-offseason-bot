package frc.robot.subsystems;

import javafx.scene.media.Media;
import javafx.scene.media.MediaPlayer;
import java.io.File;
import java.util.HashMap;

import edu.wpi.first.wpilibj.Filesystem;

public class Audio {
  public static String START = "start.mp3";
  public static String CRASH = "crash.mp3";
  public static String ENDGAME = "endgame.mp3";
  public static String GAME_OVER = "game_over.mp3";
  public static String WATER_SOUNDS = "water_sounds.mp3";
  public static String WATER_ENDGAME = "water_endgame.mp3";

  private HashMap<String, MediaPlayer> audioMap = new HashMap<>();

  private static Audio instance = new Audio();
  public static Audio getInstance() {
    return instance;
  }

  private Audio() {
    addAudio(START);
    addAudio(CRASH);
    addAudio(ENDGAME);
    addAudio(GAME_OVER);
    addAudio(WATER_SOUNDS);
    addAudio(WATER_ENDGAME);
  }

  /**
   * Adds an audio file to the audio map
   * @param name
   */
  public void addAudio(String name) {
    File deployDirectory = Filesystem.getDeployDirectory();
    var f = new File(deployDirectory, name);
    if (!f.exists()) {
      System.out.println("Audio file not found: " + name);
      return;
    }

    Media sound = new Media(f.toURI().toString());
    audioMap.put(name, new MediaPlayer(sound));
  }

  /**
   * Plays the audio file with the specified name
   * @param name
   */
  public void playAudio(String name) {
    if (audioMap.containsKey(name)) {
      if (audioMap.get(name).getStatus() != MediaPlayer.Status.PLAYING)
        audioMap.get(name).play();
    }
  }

  /**
   * Plays the audio file with the specified name
   * @param name
   */
  public void playAudio(String name, double volume) {
    if (audioMap.containsKey(name)) {
      if (audioMap.get(name).getStatus() != MediaPlayer.Status.PLAYING) {
        audioMap.get(name).setVolume(volume);
        audioMap.get(name).play();
      }
    }
  }

  public void playAudioFromStart(String name, double volume) {
    if (audioMap.containsKey(name)) {
      MediaPlayer player = audioMap.get(name);
      player.stop();
      player.setVolume(volume);
      player.play();
    }
  }

  /**
   * Changes the volume to the specified level, clamps between 0 and 1
   * @param name
   * @param volume
   */
  public void changeVolume(String name, double volume) {
    if (audioMap.containsKey(name)) {
      audioMap.get(name).setVolume(volume);
    }
  }

  /**
   * Stops the audio file with the specified name
   * @param name
   */
  public void stopAudio(String name) {
    if (audioMap.containsKey(name)) {
      audioMap.get(name).stop();
    }
  }
}
