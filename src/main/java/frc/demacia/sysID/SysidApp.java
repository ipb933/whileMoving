package frc.demacia.sysID;

import javax.swing.*;
import javax.swing.filechooser.FileNameExtensionFilter;
import java.awt.*;
import java.awt.event.*;
import java.io.*;
import java.util.*;
import java.util.List;
import java.util.function.Consumer;

public class SysidApp {
    public static boolean[] kFlags = {true, true, true, false, false, false};

    public static void main(String[] args) {
        SysidMain app = new SysidMain();
        app.show();
    }
}

class SysidMain implements Consumer<File> {
    JFrame frame = new JFrame("SysID");
    FileChooserPanel fileChooser = new FileChooserPanel(this);
    DefaultListModel<MotorData> listModel = new DefaultListModel<>();
    JList<MotorData> motorList = new JList<>(listModel);
    SysidResultPanel result = new SysidResultPanel(this);
    JTextArea msgArea = new JTextArea();
    JScrollPane msgPane = new JScrollPane(msgArea, JScrollPane.VERTICAL_SCROLLBAR_ALWAYS, JScrollPane.HORIZONTAL_SCROLLBAR_NEVER);
    
    Map<String, LogReader.SysIDResults> analysisResults;
    File currentFile;

    private static SysidMain sysid = null;

    public SysidMain() {
        sysid = this;
        frame.setSize(1024,800);
        frame.setMinimumSize(new Dimension(1024,800));
        frame.setLocation(300,200);
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        var pane = frame.getContentPane();
        pane.setLayout(new GridBagLayout());
        
        motorList.setSelectionMode(ListSelectionModel.SINGLE_SELECTION);
        motorList.setMinimumSize(new Dimension(300,400));
        motorList.setBorder(BorderFactory.createEtchedBorder());
        
        motorList.addListSelectionListener(e -> {
            if (!e.getValueIsAdjusting()) {
                MotorData selected = motorList.getSelectedValue();
                if (selected != null) {
                    msg("Selected: " + selected.displayName);
                    result.updateDisplay(selected);
                }
            }
        });
        
        pane.add(fileChooser, new GridBagConstraints(0, 0, 1, 1, 1, 1, GridBagConstraints.CENTER, GridBagConstraints.HORIZONTAL, new Insets(5, 5, 5, 0), 5, 5));
        pane.add(motorList, new GridBagConstraints(0, 1, 1, 1, 1, 1, GridBagConstraints.CENTER, GridBagConstraints.BOTH, new Insets(5, 5, 5, 0), 5, 5));
        pane.add(msgPane, new GridBagConstraints(1, 0, 1, 1, 1, 1, GridBagConstraints.CENTER, GridBagConstraints.BOTH, new Insets(5, 5, 5, 0), 5, 5));
        pane.add(result, new GridBagConstraints(1, 1, 1, 1, 1, 1, GridBagConstraints.CENTER, GridBagConstraints.BOTH, new Insets(5, 5, 5, 0), 5, 5));
        frame.pack();
    }

    public void show() {
        frame.setVisible(true);
    }

    public static void msg(String msg) {
        if(sysid != null) {
            sysid.msgArea.append(msg + "\n");
            sysid.msgArea.setCaretPosition(sysid.msgArea.getDocument().getLength());
        }
    }

    @Override
    public void accept(File file) {
        System.out.println("File set to " + file);
        this.currentFile = file;
        performFullAnalysis();
    }

    public void performFullAnalysis() {
        if (currentFile == null) return;
        
        MotorData currentlySelected = motorList.getSelectedValue();
        String selectedName = (currentlySelected != null) ? currentlySelected.fullName : null;

        try {
            MotorData.motors.clear();
            listModel.clear();
            
            analysisResults = LogReader.getResult(currentFile.getAbsolutePath());
            
            msg("Found " + analysisResults.size() + " groups.");
            
            List<String> sortedNames = new ArrayList<>(analysisResults.keySet());
            Collections.sort(sortedNames);

            int indexToSelect = 0;
            int i = 0;
            
            for(String groupName : sortedNames) {
                MotorData motorData = new MotorData();
                motorData.fullName = groupName;
                
                String[] parts = groupName.split(":");
                motorData.displayName = parts.length > 0 ? parts[0] : groupName;
                
                motorData.sysidResult = analysisResults.get(groupName);
                MotorData.motors.add(motorData);
                listModel.addElement(motorData);
                
                if(selectedName != null && selectedName.equals(groupName)) {
                    indexToSelect = i;
                }
                i++;
                msg("Added: " + motorData.displayName);
            }
            
            if (!listModel.isEmpty()) {
                motorList.setSelectedIndex(indexToSelect);
                result.updateDisplay(motorList.getSelectedValue());
            }
            
        } catch (Exception e) {
            msg("Error: " + e);
            e.printStackTrace();
            fileChooser.field.setText("");
        }
    }

    public MotorData getMotor() {
        return motorList.getSelectedValue();
    }
}

class FileChooserPanel extends JPanel implements ActionListener {
    JButton button;
    JTextField field;
    JFileChooser chooser;
    Consumer<File> consumer;

    public FileChooserPanel(Consumer<File> consumer) {
        super(new FlowLayout(FlowLayout.LEFT, 10, 5));
        this.consumer = consumer;
        button = new JButton("File:");
        field = new JTextField(50);
        field.setEditable(false);
        button.addActionListener(this);
        chooser = new JFileChooser(System.getProperty("user.dir"));
        chooser.setFileFilter(new FileNameExtensionFilter("wpi log", "wpilog"));
        add(button);
        add(field);
    }

    @Override
    public void actionPerformed(ActionEvent e) {
        int res = chooser.showOpenDialog(null);
        if(res == JFileChooser.APPROVE_OPTION) {
            File file = chooser.getSelectedFile();
            field.setText(file.getAbsolutePath());
            if(consumer != null) {
                consumer.accept(file);
            }
        }
    }
}

class SysidResultPanel extends JPanel {
    public static int nK = KTypes.values().length;
    JCheckBox[] checkBoxes;
    JLabel[][] k;
    JButton applyButton;
    JComboBox<String> kpSelector;
    
    JLabel[] velLabels = {new JLabel("Range"), new JLabel("Low"), new JLabel("Mid"), new JLabel("High")};
    JLabel[] countLabels = {new JLabel("Count"), new JLabel("0"), new JLabel("0"), new JLabel("0")};
    JLabel[] avgErrorLabels = {new JLabel("Avg Err %"), new JLabel("0"), new JLabel("0"), new JLabel("0")};
    JLabel[] maxErrorLabels = {new JLabel("Max Err %"), new JLabel("0"), new JLabel("0"), new JLabel("0")};
    JLabel[] kpLabels = {new JLabel("0"), new JLabel("0"), new JLabel("0")};
    
    SysidMain app;

    public SysidResultPanel(SysidMain app) {
        super(new GridLayout(nK + 6, 4, 5, 5));
        this.app = app;

        for(JLabel l : velLabels) add(l);
        for(JLabel l : countLabels) add(l);
        for(JLabel l : avgErrorLabels) add(l);
        for(JLabel l : maxErrorLabels) add(l);

        checkBoxes = new JCheckBox[nK];
        k = new JLabel[nK][3];
        
        for(int i = 0; i < nK; i++) {
            KTypes type = KTypes.values()[i];
            checkBoxes[i] = new JCheckBox(type.name());
            checkBoxes[i].setSelected(SysidApp.kFlags[i]);
            
            int finalI = i;
            checkBoxes[i].addActionListener(e -> {
                SysidApp.kFlags[finalI] = checkBoxes[finalI].isSelected();
                app.performFullAnalysis();
            });

            add(checkBoxes[i]);
            for(int j=0; j<3; j++){
                k[i][j] = new JLabel("0");
                add(k[i][j]);
            }
        }

        JPanel kpPanel = new JPanel(new FlowLayout(FlowLayout.LEFT, 0, 0));
        kpPanel.add(new JLabel("KP Type: "));
        String[] opts = {"1", "2"};
        kpSelector = new JComboBox<>(opts);
        kpSelector.addActionListener(e -> updateDisplay(app.getMotor()));
        kpPanel.add(kpSelector);
        
        add(kpPanel);

        for(JLabel l : kpLabels) add(l);

        applyButton = new JButton("Calc");
        add(applyButton);
        applyButton.addActionListener(e -> app.performFullAnalysis());
    }
    
    public void updateDisplay(MotorData motorData) {
        if (motorData == null || motorData.sysidResult == null) {
            clearDisplay();
            return;
        }
        
        LogReader.SysIDResults result = motorData.sysidResult;
        LogReader.BucketResult[] buckets = {result.slow, result.mid, result.high};
        
        boolean type1 = kpSelector.getSelectedIndex() == 0;

        for(int rangeIdx = 0; rangeIdx < 3; rangeIdx++) {
            LogReader.BucketResult bucket = buckets[rangeIdx];
            
            if(bucket != null) {
                k[KTypes.KS.ordinal()][rangeIdx].setText(String.format("%.5f", bucket.ks));
                k[KTypes.KV.ordinal()][rangeIdx].setText(String.format("%.5f", bucket.kv));
                k[KTypes.KA.ordinal()][rangeIdx].setText(String.format("%.5f", bucket.ka));
                k[KTypes.KG.ordinal()][rangeIdx].setText(String.format("%.5f", bucket.kg));
                k[KTypes.KSIN.ordinal()][rangeIdx].setText(String.format("%.5f", bucket.ksin));
                k[KTypes.KV2.ordinal()][rangeIdx].setText(String.format("%.5f", bucket.kv2));
                
                countLabels[rangeIdx+1].setText(Integer.toString(bucket.points));
                avgErrorLabels[rangeIdx+1].setText(String.format("%.2f%%", bucket.avgError*100));
                maxErrorLabels[rangeIdx+1].setText(String.format("%.2f%%", bucket.maxError*100));
                
                double kp = 0;
                double ka = Math.max(Math.abs(bucket.ka), 0.0001);
                
                if (type1) {
                    kp = (2.0 * bucket.kv) / ka;
                } else {
                    kp = (bucket.kv * bucket.kv) / (4.0 * ka);
                }
                
                kpLabels[rangeIdx].setText(String.format("%.5f", kp));
                
            } else {
                setColumnToNA(rangeIdx);
            }
        }
    }
    
    private void setColumnToNA(int rangeIdx) {
        for(int i = 0; i < nK; i++) k[i][rangeIdx].setText("-");
        countLabels[rangeIdx+1].setText("0");
        avgErrorLabels[rangeIdx+1].setText("-");
        maxErrorLabels[rangeIdx+1].setText("-");
        kpLabels[rangeIdx].setText("-");
    }

    private void clearDisplay() {
        for(int i = 0; i < nK; i++) {
            for(int j = 0; j < 3; j++) {
                k[i][j].setText("0");
            }
        }
        for(int i = 1; i < 4; i++) {
            countLabels[i].setText("0");
            avgErrorLabels[i].setText("0");
            maxErrorLabels[i].setText("0");
            kpLabels[i-1].setText("0");
        }
    }
}

class MotorData {
    static List<MotorData> motors = new ArrayList<>();
    String fullName = "Motor";
    String displayName = "Motor";
    LogReader.SysIDResults sysidResult;

    @Override
    public String toString() {
        return displayName;
    }
}

enum KTypes {KS, KV, KA, KG, KSIN, KV2}